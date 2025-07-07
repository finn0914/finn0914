"""
Copied from repo `drone_core main c0fa0af`.

Sending whole messages through TCP/IP.
TODO References
"""

import socket
import queue
import threading

import struct
import pickle

from util.log_no_ros import *


MSG_SIZE_SIZE = 4 # Size of the "msg_size" field is 4 bytes (unsigned int).

def _bytearray_extract_front(bytearr, size):
    data = bytearr[:size]
    del bytearr[:size]
    return data

class TCPClient:
    # seconds
    SEND_TIMEOUT = 0.1
    RECV_TIMEOUT = 0.1

    def __init__(self):
        self.sock = None
        self.send_queue = queue.Queue()
        self.recv_queue = queue.Queue()
        self.send_thread = None
        self.recv_thread = None
        self.is_socket_closed = False

    def connect(self, server_host, server_port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((server_host, server_port))
    
    def accept(self, sock):
        self.sock = sock
    
    def run(self):
        if self.is_socket_closed:
            raise RuntimeError("attempt to run after closing")
        self.sock.settimeout(TCPClient.RECV_TIMEOUT)
        self.send_thread = threading.Thread(target=self._send_thread_target)
        self.send_thread.start()
        self.recv_thread = threading.Thread(target=self._recv_thread_target)
        self.recv_thread.start()
    
    # Silently fail if socket is closed.
    def send(self, msg):
        if self.is_socket_closed:
            return
        self.send_queue.put(msg)

    def recv(self):
        try:
            return self.recv_queue.get_nowait()
        except queue.Empty:
            return None

    def close(self):
        self.is_socket_closed = True
        self.sock.close()
    
    def is_closed(self):
        return self.is_socket_closed
    
    def _send_thread_target(self):
        msg = None
        while not self.is_socket_closed:
            try:
                msg = self.send_queue.get(block=True, timeout=TCPClient.SEND_TIMEOUT)
            except:
                continue
            msg_data = pickle.dumps(msg)

            try:
                self.sock.send(struct.pack("I", len(msg_data)) + msg_data)
            except Exception as e:
                logwarn("[TCP] Exception, closing socket: %s"%e)
                self.close()

    def _recv_thread_target(self):
        buffer = bytearray()
        while not self.is_socket_closed:
            # 1. Receive message size.
            while len(buffer) < MSG_SIZE_SIZE:
                # Don't add this condition to "while".
                # Otherwise, when the loop brakes, buffer might not be filled.
                if self.is_socket_closed:
                    return
                try:
                    recv_data = self.sock.recv(MSG_SIZE_SIZE)
                    if len(recv_data) == 0:
                        self.is_socket_closed = True
                        return
                    buffer.extend(recv_data)
                except:
                    continue
            msg_size = struct.unpack("I", _bytearray_extract_front(buffer, MSG_SIZE_SIZE))[0]

            # 2. Receive message body.
            while len(buffer) < msg_size:
                # Don't add this condition to "while".
                # Otherwise, when the loop brakes, buffer might not be filled.
                if self.is_socket_closed:
                    return
                try:
                    recv_data = self.sock.recv(msg_size)
                    if len(recv_data) == 0:
                        self.is_socket_closed = True
                        return
                    buffer.extend(recv_data)
                except:
                    continue
            msg = pickle.loads(_bytearray_extract_front(buffer, msg_size))
            self.recv_queue.put(msg)

class TCPServer:
    LISTEN_TIMEOUT = 1       # seconds
    TIMEOUT_LOG_DIVIDER = 10 # log once every n timeouts

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listen_thread = None
        self.is_socket_closed = False
        self.clients = dict()
    
    # Returns false on error.
    def try_bind(self, host, port):
        loginfo("[TCP] Binding to %s:%d..."%(host, port))
        try:
            self.sock.bind((host, port))
        except Exception as e:
            logerr("[TCP] Exception: %s"%e)
            return False
        self.sock.listen() # Become a server socket.
        self.sock.settimeout(TCPServer.LISTEN_TIMEOUT)
        loginfo("[TCP] Bound to %s:%d..."%(host, port))
        return True
       
    def run(self):
        if self.is_socket_closed:
            raise RuntimeError("attempt to run after closing")
        self.listen_thread = threading.Thread(target=self._listen)
        self.listen_thread.start()

    def close(self):
        self.is_socket_closed = True
        self.sock.close()    
        for client in self.clients.values():
            client.close()
        
    def remove_client(self, host):
        if host not in self.clients.keys():
            logwarn("Host %s does not exist!"%s)
            return
        self.clients[host].close()
        del self.clients[host]

    def _listen(self):
        connection_timeout_counter = 0
        loginfo("[TCP] Waiting for connection...")
        while not self.is_socket_closed:
            try:
                client_sock, addr = self.sock.accept()
            except:
                connection_timeout_counter += 1
                if connection_timeout_counter >= TCPServer.TIMEOUT_LOG_DIVIDER:
                    loginfo("[TCP] No connection in %d seconds..."%TCPServer.TIMEOUT_LOG_DIVIDER)
                    connection_timeout_counter = 0
                continue
                
            loginfo("[TCP] Connection from %s"%str(addr))
            c = TCPClient()
            c.accept(client_sock)
            c.run()
            host, port = addr
            self.clients[host] = c
            