from groves_kf import KF

kf = KF([10000000.0,10000000.0,10000000.0],[0,0,0],[0.0,0.0,0.0])

def update(kf, pos, vel ,n):
    for i in range(n):
        kf.update_imu([[0,0,0],[0,0,0]])
        kf.update_gps(pos,vel, 1, 1)
        print(kf.est_pos_ecef, kf.est_vel_ecef)

pos = [10000000.0,10000000.0,10000000.0]
vel = [0,0,0]

update(kf, pos, vel, 10)

