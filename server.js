const WebSocket = require('ws');
const dmxnet = require('dmxnet');

// CONFIG: Must match your ESP32
const ESP32_IP = "10.0.0.151"; 
const UNIVERSE = 1;

// 1. Setup Art-Net Sender
const artnet = new dmxnet.dmxnet({ log: { level: 'info' } });
const sender = artnet.newSender({
  ip: ESP32_IP,
  universe: UNIVERSE,
  subnet: 0,
  net: 0,
  port: 6454
});

// 2. Setup WebSocket Server (Listens for Browser)
const wss = new WebSocket.Server({ port: 8080 });

console.log(`🚀 Bridge Active. Sending Art-Net to ${ESP32_IP}`);

wss.on('connection', ws => {
  console.log("Browser connected!");

  ws.on('message', (message) => {
    // Message is a binary buffer [r, g, b, r, g, b...] (192 bytes for 8x8)
    // We map this directly to DMX channels starting at 0
    
    // Note: dmxnet handles arrays cleanly, but ensures we act on the buffer
    const data = new Uint8Array(message);
    
    // Safety check: Don't crash if we get weird data
    if(data.length > 512) return;

    // Fill the Art-Net channels
    for (let i = 0; i < data.length; i++) {
        sender.prepChannel(i, data[i]);
    }
    
    // Force the packet out immediately
    sender.transmit();
  });
});