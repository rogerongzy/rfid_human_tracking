import numpy as np
import matplotlib.pyplot as plt

# y**2 / b**2 - x**2 / a**2 = 1

# Hyperbola parameters
# a = 2  # semi-major axis, for x
# b = 1  # semi-minor axis, for y

dist_odom = 1 # = 2 * c, in meters

pt_tag = np.array([1, 1])


pt_ant4 = np.array([-0.375, 0])
pt_ant4_old = np.array([-0.375, -dist_odom])
pt_ant1 = np.array([0.375, 0])
pt_ant1_old = np.array([0.375, -dist_odom])

# a**2 + b**2 = c**2


# delta_r = b * 2, actually get from phase
delta_r_ant4 = abs(np.linalg.norm(pt_tag - pt_ant4) - np.linalg.norm(pt_tag - pt_ant4_old))
b_ant4 = delta_r_ant4 / 2
c_ant4 = dist_odom / 2
a_ant4 = np.sqrt(c_ant4**2 - b_ant4**2)

delta_r_ant1 = abs(np.linalg.norm(pt_tag - pt_ant1) - np.linalg.norm(pt_tag - pt_ant1_old))
b_ant1 = delta_r_ant1 / 2
c_ant1 = dist_odom / 2
a_ant1 = np.sqrt(c_ant1**2 - b_ant1**2)

print('--------')
print(pt_tag)
print('delta_1: ', delta_r_ant1)
print('delta_2: ', delta_r_ant4)
print('df_delta: ', delta_r_ant1 - delta_r_ant4)



# Generate x values
x = np.linspace(-5, 5, 5000)

# Calculate y values for the upper branch of the hyperbola
# x to (x + 0.375), left move; x to (x - 0.375), right move
y_ant4 = np.sqrt((((x - pt_ant4[0])**2) * (b_ant4**2)) / (a_ant4**2) + b_ant4**2) - c_ant4
y_ant1 = np.sqrt((((x - pt_ant1[0])**2) * (b_ant1**2)) / (a_ant1**2) + b_ant1**2) - c_ant1
y_0 = np.array([0] * len(x))

# Calculate y values for the lower branch of the hyperbola (do not need in case)
# y_lower = -np.sqrt(((x**2) * (b**2)) / (a**2) + b**2)


# for idx,x_ in enumerate(x):
#     if abs(y_ant1[idx] - y_ant4[idx]) < 0.01: # 0.005
#         print('------------------')
#         print(idx)
#         print(x_)
#         print(y_ant1[idx])
#         print(y_ant4[idx])




# Plot the hyperbola
plt.plot(x, y_ant4, label='Upper Branch ant4')
plt.plot(x, y_ant1, label='Upper Branch ant1')
plt.plot(x, y_0, label='Upper Branch ant1')
# plt.plot(x, y_lower, label='Lower Branch')

# assist line for Asymptote
# plt.plot([-10, 10], [-5, 5], c='red') # [-10, -5] to [10, 5], slope=0.5=b/a
# plt.plot([10, -10], [-5, 5], c='red') # [10, -5] to [-10, 5]

# Set labels and title
plt.xlabel('x')
plt.ylabel('y')
# plt.title('Hyperbola')

# Set aspect ratio to 'equal' to avoid distortion
# plt.gca().set_aspect('equal', adjustable='box')

# Add legend
plt.legend()

# Show the plot
# plt.show()
# plt.savefig('test.png')
