# Polygon Path Extrusion Tool - Version 2.0

import bpy
import math
import numpy as np
from mathutils import Matrix

class PathExtrude():
    def execute(self, extruded_curve, extrusion_path):
        # Tabulate extrusion path vertices
        vertex_list = [np.array((extrusion_path.matrix_world@r.co).to_tuple()) for r in extrusion_path.data.vertices]

        # Detect if the extrusion path is closed
        path_closed = len(extrusion_path.data.edges) == len(vertex_list)

        # Double the first vertex if closed
        if path_closed:
            vertex_list.append(vertex_list[0])

        # Calculate difference vectors for translation
        difference_list = []
        for i in range(len(vertex_list)):
            if i == 0:
                difference_list.append(vertex_list[i])
            else:
                difference_list.append(vertex_list[i]-vertex_list[i-1])

        # Calculate average normal vectors at each point
        normalized_differences = [vector/np.linalg.norm(vector) for vector in difference_list] 
        average_list = []
        for i in range(len(normalized_differences)):
            if i == 0:
                if path_closed:
                    average_list.append(normalized_differences[i+1]+normalized_differences[-1])
                else:
                    average_list.append(normalized_differences[i+1])
            elif i != len(normalized_differences)-1:
                average_list.append(normalized_differences[i+1]+normalized_differences[i])
            else:
                if path_closed:
                    average_list.append(normalized_differences[i]+normalized_differences[1])
                else:
                    average_list.append(normalized_differences[i])
        average_list = [vector/np.linalg.norm(vector) for vector in average_list] 

        # Make curve active
        bpy.context.view_layer.objects.active = extruded_curve

        # Move the extruded curve center to the first vertex of the extrusion path
        extruded_curve.location.x = vertex_list[0][0]
        extruded_curve.location.y = vertex_list[0][1]
        extruded_curve.location.z = vertex_list[0][2]

        # Tabulate the vertices as a numpy array
        extruded_curve_vertices = [(extruded_curve.matrix_world@r.co).to_tuple() for r in extruded_curve.data.vertices]
        extruded_curve_vertices = np.array(extruded_curve_vertices)

        # Calculate the normal vector of the best fit plane for the vertices
        centered_vertices = extruded_curve_vertices - np.average(extruded_curve_vertices, axis=0)
        eigenvalues, eigenvectors = np.linalg.eig(np.dot(centered_vertices.T, centered_vertices))
        initial_normal = eigenvectors[:,list(eigenvalues).index(min(abs(eigenvalues)))]

        average_list[0] = initial_normal
        normalized_differences[0] = normalized_differences[1]
        factor_list = []
        for i in range(len(normalized_differences)):
            if i == 0:
                factor_list.append(1)
            elif i != len(normalized_differences)-1:
                factor_list.append(1/math.sin(math.acos(np.dot(-1*normalized_differences[i],normalized_differences[i+1]))/2))
            else:
                factor_list.append(1)

        for i in range(1,len(vertex_list)):
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_mode(type = 'VERT')
            if i == 1:
                bpy.ops.mesh.select_all(action='SELECT')
            bpy.ops.mesh.extrude_region_move()
            bpy.ops.transform.translate(value=difference_list[i])
            orient_vectorz = np.cross(average_list[i-1],average_list[i])
            orient_vectorz /= np.linalg.norm(orient_vectorz)
            orient_vectory = average_list[i-1]
            orient_vectorx = np.cross(orient_vectorz, orient_vectory)
            orient_vectorx /= np.linalg.norm(orient_vectorx)
            orientMatrix = Matrix(((orient_vectorx[0],orient_vectory[0],orient_vectorz[0]),
                                   (orient_vectorx[1],orient_vectory[1],orient_vectorz[1]),
                                   (orient_vectorx[2],orient_vectory[2],orient_vectorz[2])))
            cos = np.dot(average_list[i-1],average_list[i])
            if bpy.app.version[0] == 2 and bpy.app.version[1] >=90:
                bpy.ops.transform.rotate(value=math.acos(cos), orient_matrix=-1*orientMatrix)
            else:
                bpy.ops.transform.rotate(value=math.acos(cos), orient_matrix=orientMatrix)
            bpy.ops.transform.resize(value=(1/factor_list[i-1],1,1), orient_matrix=orientMatrix)
            bpy.ops.transform.resize(value=(factor_list[i],1,1), orient_matrix=orientMatrix)

        bpy.ops.object.mode_set(mode = 'OBJECT')
        return extruded_curve
