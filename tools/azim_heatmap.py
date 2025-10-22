import dpkt
import numpy as np
import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("filename", help="name of .pcap file to parse")
parser.add_argument("doppler_bin", help="index of doppler bin for which heatmap is drawn")
args = parser.parse_args()

filename = args.filename
doppler_bin = int(args.doppler_bin)

header = b"\x01\x02\x03\x04"
footer = b"\x04\x03\x02\x01"
pktbuf = []

# the skeleton for getting .pcapng data
for ts, pkt in dpkt.pcapng.Reader(open(filename,'rb')):

    eth=dpkt.ethernet.Ethernet(pkt) 
    if eth.type!=dpkt.ethernet.ETH_TYPE_IP:
       continue

    ip=eth.data

    if ip.p==dpkt.ip.IP_PROTO_UDP:
        udp = ip.data
        # Skip DHCP (BOOTP) packets
        if (udp.sport in (67, 68) or udp.dport in (67, 68)):
            continue

        payload = udp.data
        if (payload != header and payload != footer and payload != 0):
            pktbuf.append(payload)

raw_data = b''.join(pktbuf)
int16_array = np.frombuffer(raw_data, dtype=np.int16)

# shape into pairs, make into complex numbers, and reshape into radarcube
complex_pairs = int16_array.reshape(-1, 2)
complex_array = complex_pairs[:, 1] + 1j * complex_pairs[:, 0]
radarcube = complex_array.reshape(4, 32, 4, 128)

N_tx, N_doppler, N_rx, N_range = radarcube.shape
N_virtual = N_tx * N_rx

#horrible way to zero bin 0 but it works :)
for i in range(len(radarcube)):
    for k in range(len(radarcube[0])):
        for m in range(len(radarcube[0][0])):
            radarcube[i][k][m][0] = 0 + 0j

# Reshape virtual antennas: (Tx * Rx = 16, Chirp, Range)
radarcube_virtual = radarcube.reshape(N_tx * N_rx, N_doppler, N_range)  # shape: (16, 32, 128)

# Doppler FFT: apply FFT over chirp axis (axis=1)
doppler_cube = np.fft.fftshift(np.fft.fft(radarcube_virtual, axis=1), axes=1)  # shape: (16, 32, 128)

azimuth_slice = doppler_cube[:, doppler_bin, :]  # shape: (16, 128)

azimuth_fft = np.fft.fftshift(np.fft.fft(azimuth_slice, axis=0), axes=0)  # shape: (16, 128)
power = np.abs(azimuth_fft)**2

# Calculate azimuth angle axis in degrees
u = np.linspace(-1, 1, N_virtual)  # spatial frequency from -1 to 1 (normalized)
angles_rad = np.arcsin(u)          
angles_deg = np.degrees(angles_rad)

# Range axis (assuming uniform range bins)
range_bins = np.arange(power.shape[1])

# Plot heatmap
plt.figure(figsize=(10, 6))
plt.imshow(power, aspect='auto', extent=[range_bins[0], range_bins[-1], angles_deg[0], angles_deg[-1]],
           origin='lower', cmap='jet')
plt.colorbar(label='Power')
plt.xlabel('Range Bin')
plt.ylabel('Azimuth Angle (degrees)')
plt.title(f'Azimuth Heatmap at Doppler Bin {doppler_bin}')
plt.show()

