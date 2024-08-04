import tf_transformations

x = 0
y = 0
z = -0.7227693868920975
w = 0.691089294788757

euler = tf_transformations.euler_from_quaternion([x, y, z, w])

print(euler)