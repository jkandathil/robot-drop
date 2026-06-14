const { app, BrowserWindow } = require('electron');
const path = require('path');
const { spawn } = require('child_process');

let pyProc = null;

function createServer() {
  const scriptPath = path.join(__dirname, 'server.py');
  const pythonPath = path.join(__dirname, '..', '.venv', 'Scripts', 'python.exe');
  
  // Pin the server's working directory to this folder: server.py opens several
  // files by relative path (calibration.json, correction_map.json, saved
  // sequences), which would otherwise land wherever the app was launched from.
  pyProc = spawn(pythonPath, [scriptPath], { cwd: __dirname });
  pyProc.stdout.on('data', (data) => console.log('Python:', data.toString()));
  pyProc.stderr.on('data', (data) => console.error('Python Error:', data.toString()));
}

function exitServer() {
  if (pyProc) {
    pyProc.kill();
    pyProc = null;
  }
}

function createWindow() {
  const win = new BrowserWindow({
    width: 1300,
    height: 800,
    backgroundColor: '#f0f2f5',
    icon: path.join(__dirname, 'favicon.ico'),
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false
    }
  });

  win.loadFile('index.html');
}

app.whenReady().then(() => {
  createServer();
  createWindow();

  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) {
      createWindow();
    }
  });
});

app.on('window-all-closed', () => {
  exitServer();
  if (process.platform !== 'darwin') {
    app.quit();
  }
});

app.on('will-quit', exitServer);
