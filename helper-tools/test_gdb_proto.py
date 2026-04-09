#!/usr/bin/env python3
"""Raw RSP protocol test — bypasses GDB to test firmware RSP stub directly."""
import serial, time, subprocess, os, sys, socket

def setup_debug():
    """Ensure hub is in debug mode with socat bridge."""
    # Kill stale socat
    os.system("fuser -k 3333/tcp 2>/dev/null")
    time.sleep(0.3)
    
    s = serial.Serial('/dev/spike-shell', 115200, timeout=2)
    time.sleep(0.3)
    s.reset_input_buffer()
    
    # Check bininfo
    s.write(b'bininfo\r')
    time.sleep(0.5)
    resp = s.read(s.in_waiting).decode(errors='replace')
    print(f"bininfo: {resp.strip()}")
    
    # Start debug
    s.reset_input_buffer()
    s.write(b'debug\r')
    time.sleep(1)
    resp = s.read(s.in_waiting).decode(errors='replace')
    print(f"debug: {resp.strip()}")
    s.close()
    
    # Start socat
    proc = subprocess.Popen(
        ['socat', 'TCP-LISTEN:3333,reuseaddr,fork',
         'FILE:/dev/spike-gdb,b115200,raw,echo=0'],
        preexec_fn=os.setsid
    )
    time.sleep(0.5)
    print(f"socat pid: {proc.pid}")
    return proc

def send_recv(sock, cmd, timeout=2.0):
    """Send an RSP packet and receive the response."""
    cksum = sum(cmd.encode()) & 0xFF
    pkt = f'${cmd}#{cksum:02x}'.encode()
    sock.sendall(pkt)
    sock.settimeout(timeout)
    resp = b''
    try:
        while True:
            d = sock.recv(4096)
            if not d:
                break
            resp += d
            # Got a complete packet?
            if b'$' in resp:
                after_dollar = resp[resp.rfind(b'$'):]
                if b'#' in after_dollar and len(after_dollar) >= after_dollar.index(b'#') + 3:
                    break
    except socket.timeout:
        pass
    return resp

def decode_pc(hexdata):
    """Decode PC from GDB 'g' response (register 15, little-endian)."""
    if len(hexdata) >= 128:
        pc_hex = hexdata[120:128]
        return int(pc_hex[6:8]+pc_hex[4:6]+pc_hex[2:4]+pc_hex[0:2], 16)
    return None

def extract_payload(resp):
    """Extract packet payload from raw RSP response."""
    resp = resp.decode(errors='replace')
    if '$' in resp:
        return resp.split('$')[1].split('#')[0]
    return resp

def run_test(sock):
    """Run the protocol test."""
    # 1. ?
    print("\n=== ? (halt reason) ===")
    r = send_recv(sock, '?')
    payload = extract_payload(r)
    print(f"  → {payload}")
    
    # 2. g — read registers
    print("\n=== g (read registers) ===")
    r = send_recv(sock, 'g')
    payload = extract_payload(r)
    pc = decode_pc(payload)
    if pc is not None:
        print(f"  PC = 0x{pc:08X}")
    
    # 3. Z0 — set breakpoint at 0x20040072
    print("\n=== Z0,20040072,2 (set SW breakpoint) ===")
    r = send_recv(sock, 'Z0,20040072,2')
    print(f"  → {extract_payload(r)}")
    
    # 4. c — continue
    print("\n=== c (continue — waiting 8s) ===")
    cksum = sum(b'c') & 0xFF
    sock.sendall(f'$c#{cksum:02x}'.encode())
    sock.settimeout(8.0)
    resp = b''
    start = time.time()
    try:
        while True:
            d = sock.recv(4096)
            if not d:
                break
            resp += d
            if b'T05' in resp or b'T02' in resp or b'S05' in resp:
                break
    except socket.timeout:
        pass
    elapsed = time.time() - start
    payload = extract_payload(resp) if resp else "(empty)"
    print(f"  elapsed: {elapsed:.3f}s")
    print(f"  → {payload}")
    
    # 5. If halted, read registers
    if b'T05' in resp or b'S05' in resp:
        print("\n=== g (registers after halt) ===")
        r = send_recv(sock, 'g')
        payload = extract_payload(r)
        pc = decode_pc(payload)
        if pc is not None:
            print(f"  PC = 0x{pc:08X}")
            if pc == 0x20040072:
                print("  ✓ CORRECT — demo ran to breakpoint!")
            elif pc == 0x20040000:
                print("  ✗ WRONG — PC still at entry (stale halt)")
            else:
                print(f"  ? unexpected PC")
    
    # 6. Continue again (should run to completion or timeout)
    print("\n=== c (continue again — waiting 3s) ===")
    cksum = sum(b'c') & 0xFF
    sock.sendall(f'$c#{cksum:02x}'.encode())
    sock.settimeout(3.0)
    resp = b''
    start = time.time()
    try:
        while True:
            d = sock.recv(4096)
            if not d:
                break
            resp += d
            if b'T05' in resp or b'T02' in resp:
                break
    except socket.timeout:
        pass
    elapsed = time.time() - start
    if resp:
        payload = extract_payload(resp)
        print(f"  elapsed: {elapsed:.3f}s, → {payload}")
        if b'T05' in resp:
            r = send_recv(sock, 'g')
            p = extract_payload(r)
            pc2 = decode_pc(p)
            if pc2: print(f"  PC = 0x{pc2:08X}")
    else:
        print(f"  timeout {elapsed:.1f}s — demo running (no more breakpoints)")
    
    # Detach
    print("\n=== D (detach) ===")
    r = send_recv(sock, 'D')
    print(f"  → {extract_payload(r)}")

if __name__ == '__main__':
    skip_setup = '--no-setup' in sys.argv
    
    if not skip_setup:
        proc = setup_debug()
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('localhost', 3333))
    time.sleep(0.5)
    
    run_test(sock)
    sock.close()
    
    if not skip_setup:
        os.killpg(os.getpgid(proc.pid), 9)
    
    print("\nDone.")
