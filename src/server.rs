//! # server — Fibre protocol TCP server for ODrive Web GUI live tuning
//!
//! Exposes a TCP listener on `127.0.0.1:8000` (or configured address) implementing
//! the Fibre RPC protocol. Enables real-time step response plotting and live parameter
//! tuning from the official ODrive Web GUI (`gui.odriverobotics.com`) or `odrivetool`.

#[cfg(feature = "std")]
use std::io::{Read, Write};
#[cfg(feature = "std")]
use std::net::{TcpListener, TcpStream};

#[cfg(feature = "std")]
use barefoc_fibre::framing::{PacketUnwrapper, PacketWrapper};
#[cfg(feature = "std")]
use barefoc_fibre::server::FibreServer;

/// Non-blocking TCP server bridging BareFOC simulation state to the ODrive Web GUI.
#[cfg(feature = "std")]
pub struct SimFibreServer {
    listener: TcpListener,
    active_stream: Option<TcpStream>,
    unwrapper: PacketUnwrapper,
    rx_payload_buf: [u8; 256],
    tx_payload_buf: [u8; 256],
    tx_frame_buf: [u8; 512],
}

#[cfg(feature = "std")]
impl SimFibreServer {
    /// Bind the Fibre TCP server to the specified network address (e.g. `"127.0.0.1:8000"`).
    pub fn bind(addr: &str) -> std::io::Result<Self> {
        let listener = TcpListener::bind(addr)?;
        listener.set_nonblocking(true)?;

        Ok(Self {
            listener,
            active_stream: None,
            unwrapper: PacketUnwrapper::new(),
            rx_payload_buf: [0u8; 256],
            tx_payload_buf: [0u8; 256],
            tx_frame_buf: [0u8; 512],
        })
    }

    /// Default server binding to `127.0.0.1:8000`.
    pub fn default_server() -> std::io::Result<Self> {
        Self::bind("127.0.0.1:8000")
    }

    /// Poll for incoming network connections and process pending Fibre RPC packets.
    ///
    /// Non-blocking: returns immediately if no new data is present, allowing
    /// seamless integration into the simulation stepping loop.
    pub fn poll<Ctx>(&mut self, server: &FibreServer<Ctx>, ctx: &mut Ctx) {
        // Check for incoming client connection if none is active:
        if self.active_stream.is_none() {
            if let Ok((stream, _peer_addr)) = self.listener.accept() {
                let _ = stream.set_nonblocking(true);
                self.unwrapper.reset();
                self.active_stream = Some(stream);
            }
        }

        // Process bytes from active connection:
        if let Some(ref mut stream) = self.active_stream {
            let mut chunk = [0u8; 128];
            match stream.read(&mut chunk) {
                Ok(0) => {
                    // Client disconnected
                    self.active_stream = None;
                }
                Ok(bytes_read) => {
                    for &byte in &chunk[..bytes_read] {
                        match self.unwrapper.push_byte(byte, &mut self.rx_payload_buf) {
                            Ok(Some(payload_len)) => {
                                // Full packet received; dispatch to FibreServer
                                let packet = &self.rx_payload_buf[..payload_len];
                                match server.process_packet(
                                    packet,
                                    &mut self.tx_payload_buf,
                                    ctx,
                                ) {
                                    Ok(Some(resp_len)) => {
                                        // Wrap response in Fibre framing and send back to client
                                        if let Ok(frame_len) = PacketWrapper::wrap(
                                            &self.tx_payload_buf[..resp_len],
                                            &mut self.tx_frame_buf,
                                        ) {
                                            let _ = stream.write_all(&self.tx_frame_buf[..frame_len]);
                                        }
                                    }
                                    Ok(None) => {}
                                    Err(_) => {}
                                }
                            }
                            Ok(None) => {}
                            Err(_) => {
                                // Framing corruption: unwrapper resets automatically
                            }
                        }
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    // No new bytes available; resume simulation
                }
                Err(_) => {
                    // Stream read error: close connection
                    self.active_stream = None;
                }
            }
        }
    }

    /// Whether a client is currently connected.
    #[inline]
    pub fn has_client(&self) -> bool {
        self.active_stream.is_some()
    }
}
