import numpy as np
import matplotlib.pyplot as plt
import csv
import pdb

# csvfile = open('results/poses_imu.txt','rb')
imu_reading = np.genfromtxt('results/poses_imu.txt', delimiter=',',skip_header=1)
imu_reading = imu_reading[:,[2,17,18,19,29,30,31]]
# print imu_reading.shape
# writer=csv.writer(open('results/imu1.csv','wb'))
# header=['timestamp','omega_x','omega_y','omega_z','alpha_x','alpha_y','alpha_z']
# writer.writerow(header)

# for i in xrange(imu_reading.shape[0]):
# 	to_write = []
# 	# import pdb; pdb.set_trace()
# 	to_write.append(str(int(imu_reading[i,0])))
# 	for j in xrange(1,7):
# 		to_write.append(imu_reading[i,j])
# 	writer.writerow(to_write)

imu_predict = np.genfromtxt('data/pose.csv',delimiter=',',skip_header=1)
imu_predict = imu_predict[:, [0, 11, 12, 13, 14, 15, 16]]
# imu_predict = np.genfromtxt('results/imu0.csv',delimiter=',',skip_header=1)
print imu_predict.shape

tr = imu_reading[:,0]-imu_reading[0,0]
tp = imu_predict[:,0]-imu_reading[0,0]

plt.plot(tr, imu_reading[:,1],color='r')
plt.plot(tp, imu_predict[:,1],color='b')
plt.title('ang vel x')
plt.show()

plt.plot(tr, imu_reading[:,2],color='r')
plt.plot(tp, imu_predict[:,2],color='b')
plt.title('ang vel y')
plt.show()

plt.plot(tr, imu_reading[:,3],color='r')
plt.plot(tp, imu_predict[:,3],color='b')
plt.title('ang vel z')
plt.show()

plt.plot(tr, imu_reading[:,4],color='r')
plt.plot(tp, imu_predict[:,4],color='b')
plt.title('lin acc x')
plt.show()

plt.plot(tr, imu_reading[:,5],color='r')
plt.plot(tp, imu_predict[:,5],color='b')
plt.title('lin acc y')
plt.show()

plt.plot(tr, imu_reading[:,6],color='r')
plt.plot(tp, imu_predict[:,6],color='b')
plt.title('lin acc z')
plt.show()