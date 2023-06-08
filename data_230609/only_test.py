import random
import numpy as np
import matplotlib.pyplot as plt
from cmath import exp

### preset point coordinate ###
ant_1 = np.array((0, 0))
ant_2 = np.array((10, 0))
ant_3 = np.array((20, 0))
ant_4 = np.array((30, 0))

pt_1 = np.array((25, 80)) # original pos



### figure initialization ###
colors = ['crimson', 'magenta', 'darkslateblue', 'violet', 'blue', 'steelblue', 'cyan', 'turquoise', 'lime', 'green', 'yellowgreen', 'yellow', 'gold', 'goldenrod', 'orangered', 'darkred', 'gray']
plt.figure(figsize=(8,8))


def generate_random_pt(centre, radius):
    new_pt = np.array((centre[0] + random.uniform(-radius,radius), centre[1] + random.uniform(-radius,radius)))
    return new_pt

def get_phasor(pt_tag):
    phasor = np.array((np.linalg.norm(pt_tag - ant_1), np.linalg.norm(pt_tag - ant_2), np.linalg.norm(pt_tag - ant_3), np.linalg.norm(pt_tag - ant_4)))
    return phasor

def phasor_normalized(phasor):
    phasor_temp = []
    for i in range(len(phasor)):
        phasor_temp.append(exp(-1j*(phasor[i] - phasor[0])))
    phasor_normalized = np.matrix(phasor_temp)
    return phasor_normalized

def cosine_similarity(complex_vec_1, complex_vec_2):
    num = np.linalg.norm(np.dot(complex_vec_1, complex_vec_2.getH()))
    den = np.linalg.norm(complex_vec_1) * np.linalg.norm(complex_vec_2)
    return num / den

def plot_candidates(pt_current, raduis, num_circular_split):
    for i in range(num_circular_split):
        pt_temp = np.array((pt_current[0] + raduis * np.cos(i * 2 * np.pi / num_circular_split), pt_current[1] + raduis * np.sin(i * 2 * np.pi / num_circular_split)))
        plt.scatter(pt_temp[0], pt_temp[1], c=colors[i], s=300)
    plt.scatter(pt_current[0], pt_current[1], c=colors[16], s=300)

def candidate_generator(pt_current, pt_measured, raduis, num_circular_split):
    candidates = []
    result = []
    phasor_measured = get_phasor(pt_measured)

    for i in range(num_circular_split):
        pt_temp = np.array((pt_current[0] + raduis * np.cos(i * 2 * np.pi / num_circular_split), pt_current[1] + raduis * np.sin(i * 2 * np.pi / num_circular_split)))
        phasor_temp = get_phasor(pt_temp)
        sim_temp = cosine_similarity(phasor_normalized(phasor_measured), phasor_normalized(phasor_temp))
        
        candidates.append(pt_temp)
        result.append(sim_temp)
    
    # add itself as a candidate, other than the circle, totally 9 candidates
    pt_temp = pt_current
    phasor_temp = get_phasor(pt_temp)
    sim_temp = cosine_similarity(phasor_normalized(phasor_measured), phasor_normalized(phasor_temp))

    candidates.append(pt_temp)
    result.append(sim_temp)


    # 1, only returning the candidate with the biggest similarity
    idx = result.index(max(result))
    plt.scatter(pt_measured[0], pt_measured[1], c=colors[idx], s=5)
    # return candidates[idx]


    # [not working well] 2, weighted sum of all 
    # result_normalize = [x / sum(result) for x in result]
    # pt_new = np.array((0, 0))
    # for i in range(len(result_normalize)):
    #     # print(candidates[i] * result_normalize[i])
    #     pt_new = pt_new + candidates[i] * result_normalize[i]
    # return pt_new


    # 3, weighted sum of outstanding some of them, by threshold (0.9, 0.95, etc.)
    # new_result = []
    # new_candidates = []
    # for i in range(len(result)):
    #     if result[i] > 0.95:
    #         new_result.append(result[i])
    #         new_candidates.append(candidates[i])

    # result_normalize = [x / sum(new_result) for x in new_result]
    # pt_new = np.array((0, 0))
    # for i in range(len(result_normalize)):
    #     # print(candidates[i] * result_normalize[i])
    #     pt_new = pt_new + new_candidates[i] * result_normalize[i]

    # return pt_new
    
    
    # 4, weighted sum of outstanding 3 some of them (preset number)
    # new_result = []
    # new_candidates = []
    # for _ in range(3):
    #     max_result = max(result)
    #     idx = result.index(max_result)
    #     new_result.append(result[idx])
    #     new_candidates.append(candidates[idx])
    #     result[idx] = 0

    # result_normalize = [x / sum(new_result) for x in new_result]
    # pt_new = np.array((0, 0))
    # for i in range(len(result_normalize)):
    #     # print(candidates[i] * result_normalize[i])
    #     pt_new = pt_new + new_candidates[i] * result_normalize[i]

    # return pt_new


    # 5, weighted sum of all, add some preset/heuristic factors on weights







if __name__ == '__main__':
    plot_candidates(pt_1, 5, 16)
    for _ in range(5000):
        new_pt = generate_random_pt(pt_1, 5)
        # print(new_pt)
        candidate_generator(pt_1, new_pt, 5, 16)

    # plt.show()
    plt.savefig('25-80.png')


    # mylist = [1,3,5,2,6,4]
    # idx_list = []
    # for _ in range(2):
    #     maxnum = max(mylist)
    #     idx = mylist.index(maxnum)
    #     mylist[idx] = 0
    #     idx_list.append(idx)

    # print(idx_list)
    # print(mylist[2])
 

    
    



 
    
    


"""
It can be seen that multiple of them approach to 0.99
[0.994074525122763, 0.9998240079916879, 0.9370650565663665, 0.7441692049198252, 0.592632403841898, 0.6532177701212211, 0.8721051411923184, 0.9953489177496571, 0.908985062713027]
candidates 1 2 8 have big values, but actually closer to candidate 1, candidate 2 has the biggest similarity
---if updated by direct replacement, candidate 2 will be chosen, [  3.53553391 103.53553391]
---if updated by synthesis, weighted sum as,
------synthesis all of them， [ 0.95284506 99.66851705]
------synthesis some bigger layer of them, distingush by threshold 0.9 [  3.1514341  100.30761284], or top 3 [  4.02254225 100.00529292]
---------why 3: 2 for mutual compensation, 1 for bias, seems to be the best
------synthesis with bonus 
Draw a hotspot graph, proofing the possibility is not equal to all candidates
"""