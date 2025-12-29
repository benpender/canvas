const opencvSrc = "https://cdn.jsdelivr.net/npm/opencv.js-webassembly@4.2.0/opencv.js";
const LOG_TAG = "trace-v8";
const opencvInitTimeoutMs = 60000;
let controlPort = null;

function sendMessage(message, transfer) {
  if (controlPort) {
    controlPort.postMessage(message, transfer);
  } else {
    postMessage(message, transfer);
  }
}

function sendMessageGlobal(message, transfer) {
  postMessage(message, transfer);
}

function postStatus(message) {
  sendMessage({ type: "status", message });
}

function postLog(message, data) {
  const tagged = `${LOG_TAG} ${message}`;
  sendMessage({ type: "log", message: tagged, data });
}

function configureModule() {
  const moduleRef = self.Module || {};
  if (!moduleRef.locateFile) {
    moduleRef.locateFile = (path) => new URL(path, opencvSrc).href;
  }
  if (!moduleRef.onAbort) {
    moduleRef.onAbort = (what) => postError(`OpenCV abort: ${what}`);
  }
  if (!moduleRef.onRuntimeInitialized) {
    moduleRef.onRuntimeInitialized = () => {
      postStatus("OpenCV runtime initialized.");
      postLog("opencv: onRuntimeInitialized");
      runtimeReady = true;
      if (runtimeReadyResolve) runtimeReadyResolve();
    };
  }
  self.Module = moduleRef;
}

let cv = null;
let cvReady = false;
let runtimeReady = false;
let runtimeReadyResolve = null;
const runtimeReadyPromise = new Promise((resolve) => {
  runtimeReadyResolve = resolve;
});
let stereo = null;
let currentAlgText = "Auto";

let matcherSettings = {
  numDisp: 128,
  blockSize: 15,
  uniq: 10,
  speckleWin: 80,
  speckleRange: 2,
  forceBM: false,
  forceSGBM: false
};

let dims = {
  videoW: 0,
  videoH: 0,
  computeW: 0,
  computeH: 0,
  gapPx: 0,
  eyeW: 0,
  eyeH: 0
};

let leftRGBA = null;
let rightRGBA = null;
let leftGray = null;
let rightGray = null;
let leftSmall = null;
let rightSmall = null;
let disp16 = null;
let mask8 = null;
let resultCount = 0;
let frameCount = 0;
let droppedFrameCount = 0;

function safeDel(mat) {
  try {
    if (mat) mat.delete();
  } catch {}
}

function clearMats() {
  safeDel(leftRGBA);
  safeDel(rightRGBA);
  safeDel(leftGray);
  safeDel(rightGray);
  safeDel(leftSmall);
  safeDel(rightSmall);
  safeDel(disp16);
  safeDel(mask8);
  leftRGBA = rightRGBA = leftGray = rightGray = leftSmall = rightSmall = disp16 = mask8 = null;
  dims = { videoW: 0, videoH: 0, computeW: 0, computeH: 0, gapPx: 0, eyeW: 0, eyeH: 0 };
}

function allocMats(videoW, videoH, computeW, computeH, gapPx) {
  clearMats();

  const eyeW = Math.floor((videoW - gapPx) / 2);
  const eyeH = videoH;

  leftRGBA = new cv.Mat(eyeH, eyeW, cv.CV_8UC4);
  rightRGBA = new cv.Mat(eyeH, eyeW, cv.CV_8UC4);
  leftGray = new cv.Mat(eyeH, eyeW, cv.CV_8UC1);
  rightGray = new cv.Mat(eyeH, eyeW, cv.CV_8UC1);
  leftSmall = new cv.Mat(computeH, computeW, cv.CV_8UC1);
  rightSmall = new cv.Mat(computeH, computeW, cv.CV_8UC1);
  disp16 = new cv.Mat(computeH, computeW, cv.CV_16S);
  mask8 = new cv.Mat(computeH, computeW, cv.CV_8UC1);

  dims = { videoW, videoH, computeW, computeH, gapPx, eyeW, eyeH };
}

function hasStereoBM() {
  return (typeof cv.StereoBM_create === "function") ||
    (cv.StereoBM && typeof cv.StereoBM.create === "function");
}

function hasStereoSGBM() {
  return (typeof cv.StereoSGBM_create === "function") ||
    (cv.StereoSGBM && typeof cv.StereoSGBM.create === "function");
}

function createStereoMatcher() {
  if (!cvReady) return;

  if (stereo) {
    safeDel(stereo);
    stereo = null;
  }

  let nd = matcherSettings.numDisp | 0;
  nd = Math.max(16, nd);
  nd = (nd + 15) & ~15;
  let bs = matcherSettings.blockSize | 0;
  if (bs % 2 === 0) bs += 1;
  bs = Math.max(5, Math.min(31, bs));

  const wantBM = matcherSettings.forceBM && !matcherSettings.forceSGBM;
  const wantSGBM = matcherSettings.forceSGBM && !matcherSettings.forceBM;

  let use = "Auto";
  if (wantBM) use = "BM";
  else if (wantSGBM) use = "SGBM";
  else use = hasStereoBM() ? "BM" : (hasStereoSGBM() ? "SGBM" : "None");

  if (use === "BM") {
    if (!hasStereoBM()) throw new Error("StereoBM not available in this OpenCV build.");
    stereo = (typeof cv.StereoBM_create === "function") ? cv.StereoBM_create(nd, bs) : cv.StereoBM.create(nd, bs);
    const tc = (fn, v) => { try { stereo[fn](v); } catch {} };
    tc("setUniquenessRatio", matcherSettings.uniq | 0);
    tc("setSpeckleWindowSize", matcherSettings.speckleWin | 0);
    tc("setSpeckleRange", matcherSettings.speckleRange | 0);
    currentAlgText = `StereoBM (nd=${nd}, bs=${bs})`;
    postLog("createStereoMatcher: BM", { nd, bs });
    return;
  }

  if (use === "SGBM") {
    if (!hasStereoSGBM()) throw new Error("StereoSGBM not available in this OpenCV build.");
    const minD = 0;
    const P1 = 8 * 1 * bs * bs;
    const P2 = 32 * 1 * bs * bs;
    const disp12MaxDiff = 1;
    const preFilterCap = 63;
    const uniquenessRatio = matcherSettings.uniq | 0;
    const speckleWindowSize = matcherSettings.speckleWin | 0;
    const speckleRangeV = matcherSettings.speckleRange | 0;
    const mode = (cv.STEREO_SGBM_MODE_SGBM_3WAY ?? 0);

    if (typeof cv.StereoSGBM_create === "function") {
      try {
        stereo = cv.StereoSGBM_create(minD, nd, bs, P1, P2, disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRangeV, mode);
      } catch {
        stereo = cv.StereoSGBM_create(minD, nd, bs);
      }
    } else {
      stereo = cv.StereoSGBM.create(minD, nd, bs);
    }

    currentAlgText = `StereoSGBM (nd=${nd}, bs=${bs})`;
    postLog("createStereoMatcher: SGBM", { nd, bs });
    return;
  }

  currentAlgText = "None";
  throw new Error("No stereo matcher available (calib3d bindings missing).");
}

function computeDThreshPx(computeW, distFt, baselineMm, fovDeg) {
  const Zm = distFt * 0.3048;
  const Bm = baselineMm / 1000.0;
  const fpx = (computeW / 2) / Math.tan((fovDeg * Math.PI / 180) / 2);
  return (fpx * Bm) / Math.max(1e-6, Zm);
}

async function waitForCv(timeoutMs = opencvInitTimeoutMs) {
  if (!cv) throw new Error("OpenCV not loaded.");
  const started = performance.now();
  const timeLeft = () => timeoutMs - (performance.now() - started);
  const withTimeout = (promise, message) => {
    const remaining = timeLeft();
    if (remaining <= 0) {
      return Promise.reject(new Error(message));
    }
    return Promise.race([
      promise,
      new Promise((_, reject) => setTimeout(() => reject(new Error(message)), remaining))
    ]);
  };

  postLog("waitForCv: start", { hasMat: typeof cv?.Mat === "function", hasThen: typeof cv?.then === "function" });

  if (cv && typeof cv.Mat === "function") return;

  if (!runtimeReady) {
    await withTimeout(runtimeReadyPromise, "OpenCV runtime init timed out.");
  }

  postLog("waitForCv: after runtime", { hasMat: typeof cv?.Mat === "function", runtimeReady });

  if (!cv || !cv.Mat) {
    if (self.cv && self.cv.Mat) {
      cv = self.cv;
    } else if (self.Module && self.Module.Mat) {
      cv = self.Module;
    }
  }

  postLog("waitForCv: resolved", { hasMat: typeof cv?.Mat === "function" });

  if (!cv || !cv.Mat) {
    postStatus("OpenCV runtime initialized, but API not detected yet.");
  }
}

async function initCv() {
  if (cvReady) return cv;
  configureModule();
  postLog("initCv: importScripts", { opencvSrc });
  postStatus("Loading OpenCV script…");
  importScripts(opencvSrc);
  postLog("initCv: importScripts done");
  postStatus("OpenCV script loaded. Creating module…");
  if (typeof self.cv === "function") {
    postLog("initCv: calling factory");
    let created = self.cv();
    postLog("initCv: factory returned", {
      type: typeof created,
      hasThen: typeof created?.then === "function",
      hasMat: typeof created?.Mat === "function"
    });
    if (created && typeof created.then === "function") {
      created.then(() => postLog("initCv: then resolved"));
    }
    if (typeof created === "function") {
      postLog("initCv: calling nested factory");
      let nested = created();
      postLog("initCv: nested returned", {
        type: typeof nested,
        hasThen: typeof nested?.then === "function",
        hasMat: typeof nested?.Mat === "function"
      });
      if (nested && typeof nested.then === "function") {
        nested = await nested;
      }
      created = nested;
    }
    cv = created;
    if (cv) self.cv = cv;
  } else {
    cv = self.cv || self.Module;
  }
  postLog("initCv: module created", { hasMat: typeof cv?.Mat === "function", hasThen: typeof cv?.then === "function" });
  postStatus(`OpenCV module created. Mat=${typeof cv?.Mat === "function" ? "yes" : "no"}`);
  await waitForCv();
  postLog("initCv: ready", { hasMat: typeof cv?.Mat === "function" });
  cvReady = true;
  return cv;
}

function postError(message) {
  sendMessage({ type: "error", message });
}

function handleFrame(data) {
  const videoW = data.videoW | 0;
  const videoH = data.videoH | 0;
  const computeW = data.computeW | 0;
  const computeH = data.computeH | 0;
  const gapPx = data.gapPx | 0;
  const offsetX = data.offsetX | 0;
  const swap = !!data.swap;
  const invert = !!data.invert;
  const debug = !!data.debug;
  const distFt = Number(data.dist);
  const baselineMm = Number(data.baseline);
  const fovDeg = Number(data.fov);
  const median = data.median | 0;

  const hasBuffer = !!data.frameBuffer;
  const hasImageData = !!data.imageData;
  if (!hasBuffer && !hasImageData) {
    postLog("frame: missing image data");
    return;
  }
  if (hasBuffer) {
    postLog("frame: buffer received", { byteLength: data.frameBuffer.byteLength, videoW, videoH });
  } else {
    postLog("frame: imageData received", { width: data.imageData.width, height: data.imageData.height });
  }

  if (
    videoW !== dims.videoW ||
    videoH !== dims.videoH ||
    computeW !== dims.computeW ||
    computeH !== dims.computeH ||
    gapPx !== dims.gapPx
  ) {
    allocMats(videoW, videoH, computeW, computeH, gapPx);
    postLog("frame: allocMats", { eyeW: dims.eyeW, eyeH: dims.eyeH, computeW, computeH, gapPx });
  }

  if (!stereo) {
    createStereoMatcher();
    sendMessage({ type: "alg", text: currentAlgText });
  }

  const t1 = performance.now();
  let frameRGBA = null;
  if (hasImageData) {
    frameRGBA = cv.matFromImageData(data.imageData);
  } else {
    const bufferView = new Uint8ClampedArray(data.frameBuffer);
    frameRGBA = new cv.Mat(videoH, videoW, cv.CV_8UC4);
    frameRGBA.data.set(bufferView);
  }
  postLog("frame: mat created", { cols: frameRGBA.cols, rows: frameRGBA.rows });

  const x0 = Math.max(0, Math.min(frameRGBA.cols - 2, offsetX));
  const xL = Math.max(0, Math.min(frameRGBA.cols - dims.eyeW, x0));
  const xR = Math.max(0, Math.min(frameRGBA.cols - dims.eyeW, x0 + dims.eyeW + dims.gapPx));

  const rectA = new cv.Rect(swap ? xR : xL, 0, dims.eyeW, dims.eyeH);
  const rectB = new cv.Rect(swap ? xL : xR, 0, dims.eyeW, dims.eyeH);

  const roiA = frameRGBA.roi(rectA);
  const roiB = frameRGBA.roi(rectB);
  roiA.copyTo(leftRGBA);
  roiB.copyTo(rightRGBA);
  roiA.delete();
  roiB.delete();

  cv.cvtColor(leftRGBA, leftGray, cv.COLOR_RGBA2GRAY);
  cv.cvtColor(rightRGBA, rightGray, cv.COLOR_RGBA2GRAY);
  cv.resize(leftGray, leftSmall, new cv.Size(computeW, computeH), 0, 0, cv.INTER_AREA);
  cv.resize(rightGray, rightSmall, new cv.Size(computeW, computeH), 0, 0, cv.INTER_AREA);

  stereo.compute(leftSmall, rightSmall, disp16);
  postLog("frame: stereo computed", { dispCols: disp16.cols, dispRows: disp16.rows });

  const dThresh = computeDThreshPx(computeW, distFt, baselineMm, fovDeg);
  const disp = disp16.data16S;
  const out = mask8.data;
  let minDisp = Infinity;
  let maxDisp = -Infinity;

  if (!debug) {
    for (let i = 0; i < out.length; i++) {
      const dpx = disp[i] / 16.0;
      if (dpx < minDisp) minDisp = dpx;
      if (dpx > maxDisp) maxDisp = dpx;
      const fg = (dpx > 0 && dpx >= dThresh);
      const v = fg ? 255 : 0;
      out[i] = invert ? (255 - v) : v;
    }

    if (median >= 3 && (median % 2) === 1) {
      try { cv.medianBlur(mask8, mask8, median); } catch {}
    }
  } else {
    let nd = matcherSettings.numDisp | 0;
    nd = Math.max(16, nd);
    nd = (nd + 15) & ~15;
    const scale = 255.0 / nd;
    for (let i = 0; i < out.length; i++) {
      const dpx = disp[i] / 16.0;
      if (dpx < minDisp) minDisp = dpx;
      if (dpx > maxDisp) maxDisp = dpx;
      const v = Math.max(0, Math.min(255, (dpx * scale) | 0));
      out[i] = invert ? (255 - v) : v;
    }
  }

  frameRGBA.delete();

  const computeMs = performance.now() - t1;
  let maskSum = 0;
  for (let i = 0; i < out.length; i++) maskSum += out[i];
  postLog("frame: mask stats", {
    minDisp,
    maxDisp,
    dThresh,
    maskAvg: out.length ? (maskSum / out.length).toFixed(2) : "n/a"
  });
  const outArray = new Uint8ClampedArray(mask8.data);
  const calcText =
    `Matcher: ${currentAlgText}\n` +
    `Distance(ft): ${distFt.toFixed(1)} | Baseline(mm): ${baselineMm} | FOV(deg): ${fovDeg}\n` +
    `Computed d_thresh ≈ ${dThresh.toFixed(2)} px @ compute-res\n` +
    `Disp(px): min=${Number.isFinite(minDisp) ? minDisp.toFixed(2) : "n/a"} max=${Number.isFinite(maxDisp) ? maxDisp.toFixed(2) : "n/a"}\n` +
    `Tip: if mask is always white/black, adjust Baseline/FOV or numDisparities/blockSize.`;

  resultCount += 1;
  if (resultCount <= 3 || (resultCount % 60) === 0) {
    postLog("frame: result", { minDisp, maxDisp, dThresh, computeMs });
  }

  sendMessage(
    {
      type: "result",
      mask: outArray.buffer,
      width: computeW,
      height: computeH,
      computeMs,
      calcText
    },
    [outArray.buffer]
  );
}

function handleMessage(event, fromPort = false) {
  const data = event.data || {};
  if (data.type !== "frame") {
    postLog("worker: message", { type: data.type, channel: fromPort ? "port" : "global" });
  }
  if (data.type === "ping") {
    postLog("worker: ping", { t: data.t, tag: data.tag, channel: fromPort ? "port" : "global" });
    sendMessage({ type: "pong", t: data.t, tag: data.tag, channel: fromPort ? "port" : "global" });
    sendMessageGlobal({ type: "pong", t: data.t, tag: `${data.tag || "ping"}-global`, channel: "global" });
    return;
  }
  if (data.type === "frame-meta") {
    postLog("worker: frame-meta", { source: data.source, channel: fromPort ? "port" : "global", meta: data.meta || {} });
    sendMessage({ type: "frame-meta-ack", source: data.source || "unknown" });
    sendMessageGlobal({ type: "frame-meta-ack", source: `${data.source || "unknown"}-global` });
    return;
  }
  if (data.type === "init") {
    (async () => {
      try {
        postLog("worker: init");
        await initCv();
        postLog("worker: posting ready");
        sendMessage({ type: "ready" });
        postLog("worker: ready posted");
        createStereoMatcher();
        sendMessage({ type: "alg", text: currentAlgText });
      } catch (err) {
        postError(err?.message || err);
      }
    })();
    return;
  }

  if (data.type === "settings") {
    postLog("worker: settings", { source: data.source, channel: fromPort ? "port" : "global", settings: data.settings });
    sendMessage({ type: "settings-ack", source: data.source || "unknown" });
    sendMessageGlobal({ type: "settings-ack", source: `${data.source || "unknown"}-global` });
    matcherSettings = { ...matcherSettings, ...data.settings };
    if (cvReady) {
      try {
        createStereoMatcher();
        sendMessage({ type: "alg", text: currentAlgText });
      } catch (err) {
        postError(err?.message || err);
      }
    }
    return;
  }

  if (data.type === "stop") {
    clearMats();
    if (stereo) {
      safeDel(stereo);
      stereo = null;
    }
    return;
  }

  if (data.type === "frame") {
    if (!cvReady) {
      droppedFrameCount += 1;
      if (droppedFrameCount <= 3 || (droppedFrameCount % 60) === 0) {
        postLog("frame: dropped (cv not ready)", { droppedFrameCount, source: data.source, channel: fromPort ? "port" : "global" });
      }
      return;
    }
    try {
      frameCount += 1;
      if (frameCount <= 3 || (frameCount % 60) === 0) {
        postLog("worker: frame message", { frameCount, channel: fromPort ? "port" : "global" });
      }
      handleFrame(data);
    } catch (err) {
      postError(err?.message || err);
    }
  }
}

let messageListenerAttached = false;
if (!messageListenerAttached) {
  self.addEventListener("message", (event) => {
    const data = event.data || {};
    if (data.type === "connect" && event.ports && event.ports[0]) {
      controlPort = event.ports[0];
      controlPort.onmessage = (evt) => {
        postLog("worker: port message", { type: evt.data?.type });
        handleMessage(evt, true);
      };
      controlPort.onmessageerror = (err) => {
        postLog("worker: port messageerror", { message: err?.message || String(err) });
      };
      controlPort.start?.();
      postLog("worker: port connected");
      postStatus("Worker connected.");
      return;
    }
    handleMessage(event);
  });
  self.addEventListener("messageerror", (event) => {
    postLog("worker: messageerror", { message: event?.message || "unknown" });
  });
  messageListenerAttached = true;
  postLog("worker: listener attached");
}
