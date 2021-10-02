import matplotlib.pylab as plt
import mpl_toolkits.mplot3d as a3
import numpy as np


class Surface:
    fileName = '../input/bee_hive_1_face.obj'
    faces = []

    def readMesh (self):
        vertices = []
        normals = []
        try:
            f = open(self.fileName, 'r')
            lines = f.readlines()
            for line in lines:
                line = line.split()
                if line[0] == 'v': # vertex
                    coord = list(map(float,line[1:]))
                    vertices.append(coord)
                elif line[0] == 'vn': # vertex normal
                    coord = list(map(float,line[1:]))
                    normals.append(coord)
                elif line[0] == 'f': # face
                    face = {'vertex':[],'normal':[]}
                    line = line[1:]
                    for el in line:
                        indicies = list(map(int,el.split('//')))
                        face['vertex'].append(vertices[indicies[0]-1])
                        face['normal'].append(normals[indicies[1]-1])
                    self.faces.append(face)
        except:
            print("Error opening the file")

def plot_faces (faces, ax, plot_normals = True):
    for face in faces:
        face['vertex'].append(face['vertex'][0])
        face['normal'].append(face['normal'][0])
        xs = []; ys =[]; zs = []
        for i in range(len(face['vertex'])):
            vertex = face['vertex'][i]
            normal = face['normal'][i]
            xs.append(vertex[0])
            ys.append(vertex[1])
            zs.append(vertex[2])
            if plot_normals:
                plot_normal (vertex, normal, ax)

        ax.plot(xs,ys,zs,color='black',linewidth='0.5')
        plt.ion()
        plt.draw()

def plot_normal (vertex, normal, ax):
    p1 = vertex
    p2 = list(np.array(vertex) + np.array(normal)*SCALE)
    xs = [p1[0], p2[0]]
    ys = [p1[1], p2[1]]
    zs = [p1[2], p2[2]]

    ax.plot(xs,ys,zs,color='red',linewidth='1.0')
    plt.ion()
    plt.draw()


if __name__ == "__main__":
    surf = Surface()
    surf.fileName = '../input/bee_hive_2_face.obj'
    surf.readMesh()

    # plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.grid(False)
    SCALE = 0.3 # normal drawing scale

    plot_faces(surf.faces, ax)

    ax.auto_scale_xyz([-3, 3], [-2.5, 2.5], [-1, 4])
    plt.show(block=False)


