/**
 * ISRO SIH 2026 - Problem Statement 26169
 * AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
 * Python Backend Bridge Client (WebSocket)
 * 
 * Facilitates bidirectional real-time communication with the Python runtime
 * (Pygame / OpenCV / filterpy / scipy) or standalone PyInstaller executable.
 */

class PythonBackendBridge {
    constructor(url = 'ws://localhost:8765') {
        this.url = url;
        this.ws = null;
        this.connected = false;
        this.listeners = new Map();
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        this.reconnectTimer = null;
    }

    connect() {
        try {
            this.ws = new WebSocket(this.url);
            
            this.ws.onopen = () => {
                this.connected = true;
                this.reconnectAttempts = 0;
                this.emit('status', { connected: true, message: 'Connected to Python Backend Engine' });
                console.log('[Bridge] Connected to Python backend at', this.url);
            };

            this.ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    if (data.type) {
                        this.emit(data.type, data.payload || data);
                    }
                    this.emit('telemetry', data);
                } catch (err) {
                    console.error('[Bridge] Error parsing backend message:', err);
                }
            };

            this.ws.onclose = () => {
                this.connected = false;
                this.emit('status', { connected: false, message: 'Python backend disconnected. Operating in autonomous browser simulation mode.' });
            };

            this.ws.onerror = (err) => {
                this.connected = false;
                this.emit('error', err);
            };
        } catch (e) {
            this.connected = false;
            this.emit('status', { connected: false, message: 'Operating in standalone browser engine mode.' });
        }
    }

    sendParameterUpdate(paramName, paramValue) {
        if (this.connected && this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify({
                type: 'PARAM_UPDATE',
                param: paramName,
                value: paramValue,
                timestamp: Date.now()
            }));
        }
    }

    sendCommand(command, args = {}) {
        if (this.connected && this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify({
                type: 'COMMAND',
                command: command,
                args: args,
                timestamp: Date.now()
            }));
        }
    }

    on(event, callback) {
        if (!this.listeners.has(event)) {
            this.listeners.set(event, []);
        }
        this.listeners.get(event).push(callback);
    }

    emit(event, data) {
        if (this.listeners.has(event)) {
            for (let cb of this.listeners.get(event)) {
                cb(data);
            }
        }
    }
}

window.PythonBackendBridge = PythonBackendBridge;
