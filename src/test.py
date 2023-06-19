import numpy as np
import math
from pyquaternion import Quaternion

if __name__ == "__main__":
    inM = [[893.7527, 0, 648.9854], [0, 895.8022, 374.8394], [0, 0, 1]]
    exM = [[1, 0, 0, 0.45], [0, -1, 0, 0.6], [0, 0, -1, 0.9]]
    posW = np.array([0.4, 0.7, 0.08, 1]).reshape(4,1)
    w2I = np.dot(inM, exM)
    posI = np.dot(w2I, posW)

    print(posI)

    exMR = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
    inv_exMR = np.linalg.inv(exMR)
    inv_exMT = np.array([0.04, -0.1, 0.1])
    inv_exM = [[ inv_exMR[0,0],  inv_exMR[0,1], inv_exMR[0,2], inv_exMT[0]],
                [ inv_exMR[1,0], inv_exMR[1,1], inv_exMR[1,2], inv_exMT[1]], 
                [ inv_exMR[2,0], inv_exMR[2,1], inv_exMR[2,2], inv_exMT[2]]]

    #inv_M = np.dot( inv_exM, np.linalg.inv(inM))
    inv_posW = np.dot(np.linalg.inv(inM), posI)

    print(inv_posW)
