const WebSocket = require('ws');
const dgram = require('dgram');
const crypto = require('crypto');

// CONFIG: Must match your ESP32
const ESP32_IP = "10.0.0.151"; 
const UNIVERSE = 1;
const SEND_FPS = 60;

// 1. Setup sACN Sender (E1.31) - unicast (raw UDP)
const udp = dgram.createSocket('udp4');
const DMX_SLOTS = 512;
const PACKET_LEN = 638;
const E131_PORT = 5568;

const ACN_ID = Buffer.from([0x41, 0x53, 0x43, 0x2d, 0x45, 0x31, 0x2e, 0x31, 0x37, 0x00, 0x00, 0x00]);
const CID = crypto.randomBytes(16);
const SOURCE_NAME = "CodexSender";

const OFF_ROOT_FLENGTH = 16;
const OFF_ROOT_VECTOR = 18;
const OFF_ROOT_CID = 22;
const OFF_FRAME_FLENGTH = 38;
const OFF_FRAME_VECTOR = 40;
const OFF_FRAME_SOURCE = 44;
const OFF_FRAME_PRIORITY = 108;
const OFF_FRAME_SEQ = 111;
const OFF_FRAME_OPT = 112;
const OFF_FRAME_UNIVERSE = 113;
const OFF_DMP_FLENGTH = 115;
const OFF_DMP_VECTOR = 117;
const OFF_DMP_TYPE = 118;
const OFF_DMP_FIRST = 119;
const OFF_DMP_INC = 121;
const OFF_DMP_COUNT = 123;
const OFF_DMP_DATA = 125;

function buildSacnPacket() {
  const pkt = Buffer.alloc(PACKET_LEN, 0);
  pkt.writeUInt16BE(0x0010, 0); // preamble
  pkt.writeUInt16BE(0x0000, 2); // postamble
  ACN_ID.copy(pkt, 4);
  pkt.writeUInt16BE(0x7000 | (PACKET_LEN - 16), OFF_ROOT_FLENGTH);
  pkt.writeUInt32BE(0x00000004, OFF_ROOT_VECTOR);
  CID.copy(pkt, OFF_ROOT_CID);

  pkt.writeUInt16BE(0x7000 | (PACKET_LEN - 38), OFF_FRAME_FLENGTH);
  pkt.writeUInt32BE(0x00000002, OFF_FRAME_VECTOR);
  pkt.write(SOURCE_NAME, OFF_FRAME_SOURCE, 64, 'ascii');
  pkt.writeUInt8(100, OFF_FRAME_PRIORITY);
  pkt.writeUInt8(0x00, OFF_FRAME_OPT);
  pkt.writeUInt16BE(UNIVERSE, OFF_FRAME_UNIVERSE);

  pkt.writeUInt16BE(0x7000 | (PACKET_LEN - 115), OFF_DMP_FLENGTH);
  pkt.writeUInt8(0x02, OFF_DMP_VECTOR);
  pkt.writeUInt8(0xa1, OFF_DMP_TYPE);
  pkt.writeUInt16BE(0x0000, OFF_DMP_FIRST);
  pkt.writeUInt16BE(0x0001, OFF_DMP_INC);
  pkt.writeUInt16BE(DMX_SLOTS + 1, OFF_DMP_COUNT);
  pkt.writeUInt8(0x00, OFF_DMP_DATA); // start code
  return pkt;
}

let sequence = 0;
const sacnPacket = buildSacnPacket();

// 2. Setup WebSocket Server (Listens for Browser)
const wss = new WebSocket.Server({ port: 8080 });

console.log(`🚀 Bridge Active. Sending sACN unicast to ${ESP32_IP}`);

let latestData = null;
let recvCount = 0;
let sendCount = 0;
let lastSeq = null;
let dropCount = 0;
let lastLog = Date.now();
setInterval(() => {
  if (!latestData) return;
  sacnPacket.writeUInt8(sequence, OFF_FRAME_SEQ);
  sequence = (sequence + 1) & 0xff;
  sacnPacket.fill(0, OFF_DMP_DATA + 1, OFF_DMP_DATA + 1 + DMX_SLOTS);
  latestData.copy(sacnPacket, OFF_DMP_DATA + 1, 0, Math.min(latestData.length, DMX_SLOTS));
  udp.send(sacnPacket, E131_PORT, ESP32_IP);
  sendCount += 1;
  const now = Date.now();
  if (now - lastLog >= 1000) {
    console.log(`rates recv_fps=${recvCount} send_fps=${sendCount} seq_drops=${dropCount}`);
    recvCount = 0;
    sendCount = 0;
    dropCount = 0;
    lastLog = now;
  }
}, 1000 / SEND_FPS);

wss.on('connection', ws => {
  console.log("Browser connected!");

  ws.on('message', (message) => {
    // Message is a binary buffer [r, g, b, r, g, b...] (192 bytes for 8x8)
    // We map this directly to DMX channels starting at 0
    
    // Note: dmxnet handles arrays cleanly, but ensures we act on the buffer
    const data = new Uint8Array(message);
    if (data.length > 513) return;
    if (data.length === 192) {
      latestData = Buffer.from(data);
    } else {
      if (data.length < 2) return;
      const seq = data[0];
      if (lastSeq !== null) {
        const diff = (seq - lastSeq - 1 + 256) % 256;
        if (diff > 0) dropCount += diff;
      }
      lastSeq = seq;
      const payload = data.subarray(1);
      if (payload.length > 512) return;
      latestData = Buffer.from(payload);
    }
    recvCount += 1;
  });
});
