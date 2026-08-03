// LiftMaster / Chamberlain commercial-operator host-link driver (Saturn / msg1210).
// See drv_liftmaster.h for the wire format and provenance.

#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../quicktick.h"
#include "../cmnds/cmd_public.h"
#include "../logging/logging.h"
#include "drv_public.h"
#include "drv_liftmaster.h"
#include "drv_uart.h"

// ---------------------------------------------------------------- config -----
#define LM_BAUD        57600
#define LM_PARITY      0        // 8N1 (0=none, 1=odd, 2=even in UART_InitUART)
#define LM_RX_RING     1024
#define LM_FRAME_MAX   256      // max on-wire frame length we handle
#define LM_PAYLOAD_MAX 255

// Door status frame (RX, LPC->us): header 01 11 02 11, payload length 16,
// direction = last payload byte: 0x01=OPEN, 0x00=CLOSED. (docs/lpc_protocol.md)
static const byte LM_STATUS_HDR[4] = { 0x01, 0x11, 0x02, 0x11 };
#define LM_STATUS_LEN 16

// OBK channel that receives the decoded door state (0=closed, 1=open).
static int g_statusChannel = 1;
// transport sequence nibble for frames we send
static byte g_txSeq = 0;
// stats
static uint32_t g_rxFrames = 0, g_rxCrcErr = 0, g_txFrames = 0, g_txAcks = 0;

// last door direction reported by the operator (-1 unknown, 0 closed, 1 open)
static int g_lastDir = -1;
// pending door command: target direction we are driving toward (-1 = none)
static int g_doorTarget = -1;
static int g_doorRetries = 0;
// guard so our own RX-driven CHANNEL_Set doesn't re-trigger a command
static int g_suppressChannelCb = 0;

static byte g_crcTable[256];

// ------------------------------------------------------------------ crc ------
static void LM_BuildCrcTable(void) {
	int i, b;
	for (i = 0; i < 256; i++) {
		int c = i;
		for (b = 0; b < 8; b++)
			c = (c & 0x80) ? (((c << 1) ^ 0x1D) & 0xFF) : ((c << 1) & 0xFF);
		g_crcTable[i] = (byte)c;
	}
}

static byte LM_Crc8(const byte *data, int len) {
	byte c = 0xAA;
	int i;
	for (i = 0; i < len; i++)
		c = g_crcTable[data[i] ^ c];
	return c;
}

// ------------------------------------------------------------- hex helpers ---
static const char LM_HEX[] = "0123456789ABCDEF";

static int LM_HexNibble(char ch) {
	if (ch >= '0' && ch <= '9') return ch - '0';
	if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
	if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
	return -1;
}

// Parse a null-terminated hex string into bytes. Returns byte count, or -1 on
// a bad/odd-length string or overflow.
static int LM_HexToBytes(const char *s, byte *out, int maxOut) {
	int n = 0;
	while (s[0] && s[1]) {
		int hi = LM_HexNibble(s[0]);
		int lo = LM_HexNibble(s[1]);
		if (hi < 0 || lo < 0 || n >= maxOut) return -1;
		out[n++] = (byte)((hi << 4) | lo);
		s += 2;
	}
	if (s[0]) return -1; // odd number of nibbles
	return n;
}

// ----------------------------------------------------------- frame builder ---
// Build '<' TYPE SEQ hex(hdr[4] + len + payload + crc) '>' into out.
// Returns frame length, or -1 on overflow / bad args.
static int LM_BuildFrame(const byte *hdr, const byte *payload, int plen,
		int seq, char typeChar, byte *out, int maxOut) {
	byte core[5 + LM_PAYLOAD_MAX];
	int clen = 0, o = 0, i;
	byte crc;

	if (plen < 0 || plen > LM_PAYLOAD_MAX) return -1;
	for (i = 0; i < 4; i++) core[clen++] = hdr[i];
	core[clen++] = (byte)plen;
	for (i = 0; i < plen; i++) core[clen++] = payload[i];
	crc = LM_Crc8(core, clen);

	// '<' + TYPE + SEQ + (clen+1 bytes * 2 hex) + '>'
	if (maxOut < 3 + (clen + 1) * 2 + 1) return -1;
	out[o++] = '<';
	out[o++] = (byte)typeChar;
	out[o++] = LM_HEX[seq & 0xF];
	for (i = 0; i < clen; i++) {
		out[o++] = LM_HEX[core[i] >> 4];
		out[o++] = LM_HEX[core[i] & 0xF];
	}
	out[o++] = LM_HEX[crc >> 4];
	out[o++] = LM_HEX[crc & 0xF];
	out[o++] = '>';
	return o;
}

static void LM_SendFrame(const byte *frame, int len) {
	int i;
	for (i = 0; i < len; i++)
		UART_SendByte(frame[i]);
	g_txFrames++;
}

// ARQ ACK: the Saturn link is reliable — the peer runs an ack-timer and
// retransmits any data frame we don't acknowledge. Reply to each received data
// frame with '<' 'K' <seqNibble> '>' (the LPC's own FUN_1005318a format) so it
// stops resending. Without this the operator floods stale retransmits and our
// door-state feedback lags/inverts.
static void LM_SendAck(char seqChar) {
	byte f[4];
	f[0] = '<'; f[1] = 'K'; f[2] = (byte)seqChar; f[3] = '>';
	LM_SendFrame(f, 4);
	g_txAcks++;
}

// Door command as a myQ TLV (docs/rtl_saturn_protocol.md): the frame payload is
//   01 <msg_id LE> <attr_id LE> <inner payload...>
// with the inner payload's last byte = direction (01=open, 00=close). This
// template is captured from THIS board's status frame; bytes 6..11 are the
// device serial and are board-specific. hdr 01 11 02 11 routes to the door
// endpoint. Verified live: injecting this flips the operator's direction latch.
static const byte LM_DOOR_HDR[4] = { 0x01, 0x11, 0x02, 0x11 };
static byte g_doorTlv[16] = {
	0x01, 0x1c, 0x00, 0x20, 0x00, 0x01,
	0x06, 0x94, 0x50, 0xea, 0x43, 0xf5,
	0x02, 0x0d, 0x01, 0x00 /* [15] = direction */
};

static void LM_SendDoor(int open) {
	byte frame[LM_FRAME_MAX];
	int flen;
	g_doorTlv[15] = open ? 0x01 : 0x00;
	flen = LM_BuildFrame(LM_DOOR_HDR, g_doorTlv, sizeof(g_doorTlv),
		g_txSeq++, 'P', frame, sizeof(frame));
	if (flen > 0) LM_SendFrame(frame, flen);
}

// Drive the door toward a direction, retransmitting (1 Hz) until the operator's
// reported state confirms it or we run out of retries.
static void LM_DoorCommand(int open) {
	g_doorTarget = open ? 1 : 0;
	g_doorRetries = 8;
	LM_SendDoor(open);
	ADDLOG_INFO(LOG_FEATURE_GENERAL, "LM door command -> %s", open ? "OPEN" : "CLOSE");
}

// -------------------------------------------------------------- RX handler ---
// inner is the null-terminated text between '<' and '>' : TYPE SEQ hexbody
static void LM_HandleInner(char *inner, int innerLen) {
	byte body[LM_FRAME_MAX];
	int bodyLen, coreLen, plen;
	byte crcGot, crcCalc, *hdr, *payload;
	char typeChar;
	int seq;

	if (innerLen < 2 + 2) return; // need TYPE SEQ + at least a byte
	typeChar = inner[0];
	seq = LM_HexNibble(inner[1]);

	bodyLen = LM_HexToBytes(inner + 2, body, sizeof(body));
	if (bodyLen < 6) return; // need hdr(4)+len(1)+crc(1) minimum

	coreLen = bodyLen - 1;           // everything but the trailing CRC
	crcGot = body[bodyLen - 1];
	crcCalc = LM_Crc8(body, coreLen);
	hdr = body;
	plen = body[4];
	payload = body + 5;

	g_rxFrames++;
	if (crcGot != crcCalc) {
		g_rxCrcErr++;
		ADDLOG_INFO(LOG_FEATURE_GENERAL,
			"LM RX BAD-CRC type=%c seq=%d hdr=%02X%02X%02X%02X len=%d got=%02X calc=%02X",
			typeChar, seq, hdr[0], hdr[1], hdr[2], hdr[3], plen, crcGot, crcCalc);
		return;
	}
	// coreLen must equal 5 (hdr+len) + plen; guard against truncation
	if (coreLen != 5 + plen) {
		ADDLOG_INFO(LOG_FEATURE_GENERAL,
			"LM RX len-mismatch hdr=%02X%02X%02X%02X declared=%d actual=%d",
			hdr[0], hdr[1], hdr[2], hdr[3], plen, coreLen - 5);
		return;
	}

	// ARQ: acknowledge received data frames ('P') so the operator stops
	// retransmitting them. Do not ACK acks/naks/keepalives.
	if (typeChar == 'P')
		LM_SendAck(inner[1]);

	ADDLOG_INFO(LOG_FEATURE_GENERAL,
		"LM RX type=%c seq=%d hdr=%02X%02X%02X%02X len=%d",
		typeChar, seq, hdr[0], hdr[1], hdr[2], hdr[3], plen);

	// Decode the known door-status frame -> channel.
	if (plen == LM_STATUS_LEN && memcmp(hdr, LM_STATUS_HDR, 4) == 0) {
		int dir = payload[plen - 1]; // 0x01=OPEN, 0x00=CLOSED
		g_lastDir = dir ? 1 : 0;
		if (g_doorTarget == g_lastDir)
			g_doorTarget = -1; // command confirmed by the operator
		g_suppressChannelCb = 1; // this is feedback, not a user command
		CHANNEL_Set(g_statusChannel, g_lastDir, 0);
		ADDLOG_INFO(LOG_FEATURE_GENERAL, "LM door state = %s (ch%d)",
			dir ? "OPEN" : "CLOSED", g_statusChannel);
	}
}

void LiftMaster_RunFrame(void) {
	char inner[LM_FRAME_MAX];
	int avail, end, i, innerLen;

	avail = UART_GetDataSize();
	// Drop bytes until a frame start '<'.
	while (avail > 0 && UART_GetByte(0) != '<') {
		UART_ConsumeBytes(1);
		avail--;
	}
	if (avail < 2) return;

	// Look for the frame terminator '>'.
	end = -1;
	for (i = 1; i < avail; i++) {
		if (UART_GetByte(i) == '>') { end = i; break; }
	}
	if (end < 0) {
		// No complete frame yet. If it's grown implausibly long without a
		// terminator, drop the stale '<' so we can resync.
		if (avail > LM_FRAME_MAX)
			UART_ConsumeBytes(1);
		return;
	}

	innerLen = end - 1; // bytes strictly between '<' and '>'
	if (innerLen >= LM_FRAME_MAX) {
		UART_ConsumeBytes(end + 1); // oversized: discard the whole frame
		return;
	}
	for (i = 0; i < innerLen; i++)
		inner[i] = (char)UART_GetByte(1 + i);
	inner[innerLen] = 0;
	UART_ConsumeBytes(end + 1); // consume through '>'

	LM_HandleInner(inner, innerLen);
}

// -------------------------------------------------------------- commands -----
// LM_Send <hdrHex8> [payloadHex] : build a proper frame (CRC added) and send it.
// e.g.  LM_Send 01110211 00112233...   (header = 4 bytes = 8 hex chars)
static commandResult_t CMD_LM_Send(const void *context, const char *cmd,
		const char *args, int flags) {
	byte hdr[4], payload[LM_PAYLOAD_MAX], frame[LM_FRAME_MAX];
	int plen = 0, flen;
	const char *hdrStr, *plStr;

	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1)
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;

	hdrStr = Tokenizer_GetArg(0);
	if (LM_HexToBytes(hdrStr, hdr, 4) != 4) {
		ADDLOG_INFO(LOG_FEATURE_GENERAL, "LM_Send: header must be 4 bytes (8 hex chars)");
		return CMD_RES_ERROR;
	}
	if (Tokenizer_GetArgsCount() >= 2) {
		plStr = Tokenizer_GetArg(1);
		plen = LM_HexToBytes(plStr, payload, sizeof(payload));
		if (plen < 0) {
			ADDLOG_INFO(LOG_FEATURE_GENERAL, "LM_Send: bad payload hex");
			return CMD_RES_ERROR;
		}
	}
	flen = LM_BuildFrame(hdr, payload, plen, g_txSeq++, 'P', frame, sizeof(frame));
	if (flen < 0) return CMD_RES_ERROR;
	LM_SendFrame(frame, flen);
	ADDLOG_INFO(LOG_FEATURE_GENERAL, "LM TX %.*s", flen, (const char *)frame);
	return CMD_RES_OK;
}

// LM_SendRaw <hex> : send literal bytes verbatim (lowest-level experimentation).
static commandResult_t CMD_LM_SendRaw(const void *context, const char *cmd,
		const char *args, int flags) {
	byte raw[LM_FRAME_MAX];
	int n, i;

	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1)
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	n = LM_HexToBytes(Tokenizer_GetArg(0), raw, sizeof(raw));
	if (n < 0) return CMD_RES_ERROR;
	for (i = 0; i < n; i++)
		UART_SendByte(raw[i]);
	ADDLOG_INFO(LOG_FEATURE_GENERAL, "LM TX raw %d bytes", n);
	return CMD_RES_OK;
}

// LM_StatusChannel <ch> : choose which OBK channel receives the door state.
static commandResult_t CMD_LM_StatusChannel(const void *context, const char *cmd,
		const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1)
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	g_statusChannel = Tokenizer_GetArgInteger(0);
	ADDLOG_INFO(LOG_FEATURE_GENERAL, "LM status channel = %d", g_statusChannel);
	return CMD_RES_OK;
}

// LM_Door <0|1> : command the door closed(0)/open(1) with retry-until-confirmed.
static commandResult_t CMD_LM_Door(const void *context, const char *cmd,
		const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1)
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	LM_DoorCommand(Tokenizer_GetArgInteger(0) ? 1 : 0);
	return CMD_RES_OK;
}

// Channel callback: when the status channel is set externally (OBK web UI
// toggle, MQTT, script) drive the door to match. Ignore feedback writes we make
// ourselves from the RX decode (guarded by g_suppressChannelCb).
void LiftMaster_OnChannelChanged(int channel, int value) {
	if (channel != g_statusChannel) return;
	if (g_suppressChannelCb) { g_suppressChannelCb = 0; return; }
	if (value == g_lastDir) return; // already there
	LM_DoorCommand(value ? 1 : 0);
}

// ----------------------------------------------------------- lifecycle -------
void LiftMaster_Init(void) {
	LM_BuildCrcTable();
	UART_InitUART(LM_BAUD, LM_PARITY, false);
	UART_InitReceiveRingBuffer(LM_RX_RING);

	//cmddetail:{"name":"LM_Send","args":"[hdrHex8][payloadHex]",
	//cmddetail:"descr":"Build a msg1210/Saturn frame (CRC-8 added) and send it to the door board.",
	//cmddetail:"fn":"CMD_LM_Send","file":"driver/drv_liftmaster.c","requires":""}
	CMD_RegisterCommand("LM_Send", CMD_LM_Send, NULL);
	//cmddetail:{"name":"LM_SendRaw","args":"[hex]",
	//cmddetail:"descr":"Send literal bytes on the host UART (raw experimentation).",
	//cmddetail:"fn":"CMD_LM_SendRaw","file":"driver/drv_liftmaster.c","requires":""}
	CMD_RegisterCommand("LM_SendRaw", CMD_LM_SendRaw, NULL);
	//cmddetail:{"name":"LM_StatusChannel","args":"[channel]",
	//cmddetail:"descr":"Set the OBK channel that receives the decoded door state (0=closed,1=open).",
	//cmddetail:"fn":"CMD_LM_StatusChannel","file":"driver/drv_liftmaster.c","requires":""}
	CMD_RegisterCommand("LM_StatusChannel", CMD_LM_StatusChannel, NULL);
	//cmddetail:{"name":"LM_Door","args":"[0|1]",
	//cmddetail:"descr":"Command the door closed(0)/open(1); retransmits until the operator confirms.",
	//cmddetail:"fn":"CMD_LM_Door","file":"driver/drv_liftmaster.c","requires":""}
	CMD_RegisterCommand("LM_Door", CMD_LM_Door, NULL);

	ADDLOG_INFO(LOG_FEATURE_GENERAL,
		"LiftMaster (Saturn/msg1210) driver started @ %d 8N1, status->ch%d",
		LM_BAUD, g_statusChannel);
}

void LiftMaster_RunEverySecond(void) {
	// Retransmit a pending door command (1 Hz) until the operator's reported
	// state confirms it or retries are exhausted. Cleared in the RX decode.
	if (g_doorTarget >= 0) {
		if (g_doorRetries-- > 0) {
			LM_SendDoor(g_doorTarget);
		} else {
			ADDLOG_INFO(LOG_FEATURE_GENERAL,
				"LM door command gave up (no confirm), target=%d last=%d",
				g_doorTarget, g_lastDir);
			g_doorTarget = -1;
		}
	}
	if ((g_rxFrames | g_txFrames) &&
		(g_rxFrames % 20 == 0)) {
		ADDLOG_DEBUG(LOG_FEATURE_GENERAL,
			"LM stats rx=%u crcErr=%u tx=%u ack=%u", g_rxFrames, g_rxCrcErr, g_txFrames, g_txAcks);
	}
}

void LiftMaster_Shutdown(void) {
	// UART is shared infrastructure; nothing to free here.
}
