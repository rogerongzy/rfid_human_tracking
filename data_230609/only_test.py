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
        plt.scatter(pt_temp[0], pt_temp[1], c=colors[i], s=200)
    plt.scatter(pt_current[0], pt_current[1], c=colors[16], s=200)

def candidate_generator(pt_current, pt_measured, raduis, num_circular_split):
    candidates = []
    result = []
    phasor_measured = get_phasor(pt_measured)

    for i in range(num_circular_split):
        # pt_temp = np.array((pt_current[0] + raduis * np.cos(i * 2 * np.pi / num_circular_split), pt_current[1] + raduis * np.sin(i * 2 * np.pi / num_circular_split))) # circulate

        # pt_temp = np.array((pt_current[0] + random.uniform(-raduis, raduis), pt_current[1] + random.uniform(-raduis, raduis))) # random
        pt_temp = generate_random_pt(pt_current, raduis) # random

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
    # print(result)

    # 1, only returning the candidate with the biggest similarity
    idx = result.index(max(result))
    # plt.scatter(pt_measured[0], pt_measured[1], c=colors[idx], s=5)
    return candidates[idx]


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
    
    
    # 4, weighted sum of outstanding 2 some of them (preset number)
    # new_result = []
    # new_candidates = []
    # new_idx = []
    # for _ in range(2):
    #     max_result = max(result)
    #     idx = result.index(max_result)
    #     new_result.append(result[idx])
    #     new_candidates.append(candidates[idx])
    #     result[idx] = 0
    #     new_idx.append(idx)
    
    # # for temp_idx in new_idx:
    # #     plt.scatter(candidates[temp_idx][0], candidates[temp_idx][1], c=colors[temp_idx], s=400)

    # result_normalize = [x / sum(new_result) for x in new_result]
    # pt_new = np.array((0, 0))
    # for i in range(len(result_normalize)):
    #     # print(candidates[i] * result_normalize[i])
    #     pt_new = pt_new + new_candidates[i] * result_normalize[i]
    # # plt.scatter(pt_new[0], pt_new[1], c='red', s=20, label='synthesis')
    # return pt_new


    # 5, weighted sum of all, add some preset/heuristic factors on weights


def plot_hotspot():
    plt.figure(figsize=(8,8))
    plot_candidates(pt_1, 5, 16)

    for _ in range(5000):
        new_pt = generate_random_pt(pt_1, 5)
        # print(new_pt)
        candidate_generator(pt_1, new_pt, 5, 16)

    plt.show()
    # plt.savefig('25-80.png')



###### phasor differenciate ######

def candidate_generator_df(pt_current, pt_measured, raduis, num_circular_split):
    candidates = []
    result = []
    phasor_current = get_phasor(pt_current)
    phasor_measured = get_phasor(pt_measured)

    for i in range(num_circular_split):
        pt_temp = np.array((pt_current[0] + raduis * np.cos(i * 2 * np.pi / num_circular_split), pt_current[1] + raduis * np.sin(i * 2 * np.pi / num_circular_split)))
        phasor_temp = get_phasor(pt_temp)
        
        # sim_temp = cosine_similarity(phasor_normalized(phasor_measured), phasor_normalized(phasor_temp)) # direct distance/phase
        sim_temp = cosine_similarity(phasor_normalized(phasor_measured - phasor_current), phasor_normalized(phasor_temp - phasor_current)) # differenciate
        
        # print('-------------------------------')
        # print(phasor_current)

        candidates.append(pt_temp)
        result.append(sim_temp)
    
    # add itself as a candidate, other than the circle, totally 9 candidates
    pt_temp = pt_current
    phasor_temp = get_phasor(pt_temp)
    sim_temp = cosine_similarity(phasor_normalized(phasor_measured - phasor_current), phasor_normalized(phasor_temp - phasor_current))

    candidates.append(pt_temp)
    result.append(sim_temp)

    idx = result.index(max(result))
    # plt.scatter(pt_measured[0], pt_measured[1], c=colors[idx], s=5)
    return candidates[idx]

    
def plot_hotspot_df():
    plt.figure(figsize=(8,8))
    plot_candidates(pt_1, 5, 16)

    for _ in range(5000):
        new_pt = generate_random_pt(pt_1, 5)
        # print(new_pt)
        candidate_generator_df(pt_1, new_pt, 5, 16)
    
    plt.show()



###### complete simulation ######
def func(x):
    return x**2


def generate_trajectory():
    # generate a trajectory and sample at certain time-stamp
    # from (0, 40) to (40, 80)
    y = np.linspace(0, 40, 50) # len = 50, range = (0, 40)
    x = func(y) / 40
    return x, y + 40


def simulation_process():
    X, Y = generate_trajectory()
    plt.figure(figsize=(12,12))
    plt.plot(X, Y)
    plt.scatter(X, Y, c='red', s=5)
    current_pt = np.array((0, 40))

    x_set = [X[0]]
    y_set = [Y[0]]

    for i in range(len(X)-1):
        measure_pt = np.array((X[i+1], Y[i+1]))
        
        temp_pt = candidate_generator(current_pt, measure_pt, 2, 100)

        current_pt = temp_pt

        x_set.append(temp_pt[0])
        y_set.append(temp_pt[1])
        # plt.scatter(temp_pt[0], temp_pt[1], c='blue', s=5)

    plt.plot(x_set, y_set, c='blue')
    
    plt.show()


def synthesis_scope():
    plt.figure(figsize=(8,8))
    temp_pt = generate_random_pt(pt_1, 5)
    plt.scatter(temp_pt[0], temp_pt[1], c='blue', s=20, label='current ground truth')

    
    # candidate_generator(pt_1, temp_pt, 5, 16)
    # plt.legend()



if __name__ == '__main__':
    # plot_hotspot()

    # plot_hotspot_df()

    simulation_process()


    

 
    
    


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

Draw all the candidate, draw the complete curve plot on the complete simulation, inspect whether the strip-skipping effect will happen.

230612:
The 'Monte Carlo' version:
It seems better. However, the further distance will still influence the accuracy of the matching.
It can alleviate the strip-skipping effect to some degree, but still exist.
The parameter 'radius' should be big enough to cover the biggest speed of possible tag movement.
Use the best 2 of them and synthesis have a quite good performance. 
Searching space should be circle but not square.
Todo:
Number of candidates/particles.
Heuristic radius.
"""