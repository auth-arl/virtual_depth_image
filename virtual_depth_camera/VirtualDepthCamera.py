import os
import time
import warnings

import numpy as np

import glfw

from OpenGL.GL import *
from OpenGL.GL import glVertexAttribPointer
from OpenGL.GL.shaders import compileProgram, compileShader

import pyrr
from stl import mesh
import yaml
import transforms3d

try:
    from virtual_depth_camera.virtual_depth_camera import gl_shaders
except:
    from . import gl_shaders


class GraphicObject():

    def __init__(self, filename, position_loc, model_loc, origin_loc, origin_tf, scaling=1.0):
        temp_mesh = mesh.Mesh.from_file(
            filename,
            speedups=False)
        self.vertices = temp_mesh.vectors

        self.VAO = glGenVertexArrays(1)
        # bind VAO to pass vertices
        glBindVertexArray(self.VAO)

        # TODO: remove this from members.
        #  VBO will most probably be automatically binded in VAO....
        #  So no need to keepit as member.
        self.VBO = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, self.VBO)
        glBufferData(GL_ARRAY_BUFFER,
                     self.vertices.nbytes,
                     self.vertices,
                     GL_STATIC_DRAW)

        # -- uncomment this in case of Indexed Vertex format
        # self.EBO = glGenBuffers(1)
        # glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.EBO)
        # glBufferData(GL_ELEMENT_ARRAY_BUFFER, self.indices.nbytes, self.indices,  GL_STATIC_DRAW)

        glEnableVertexAttribArray(position_loc)
        glVertexAttribPointer(position_loc, 3,
                              GL_FLOAT, GL_FALSE,
                              0, ctypes.c_void_p(0))

        self.origin_loc = origin_loc
        self.model_loc = model_loc
        self.origin_tf = origin_tf
        self.origin_tf[3, 3] = scaling
        # Unbind VAO, after vertices are passed
        glBindVertexArray(0)

    def draw(self, model_tf):
        # binds the VAO that contains the GrObject's vertices, draws the triangles and unbinds.
        glBindVertexArray(self.VAO)

        #  update model_tf
        glUniformMatrix4fv(self.model_loc,  1, GL_FALSE, model_tf.transpose())
        glUniformMatrix4fv(self.origin_loc, 1, GL_FALSE, self.origin_tf.transpose())

        glDrawArrays(GL_TRIANGLES, 0, self.vertices.shape[0] * 3 * 3)
        # -- uncomment this in case of Indexed Vertex format
        # glDrawArrays(GL_TRIANGLE_STRIP, 0, self.vertices.shape[0])
        glBindVertexArray(0)


class VirtualDepthCamera:
    """
    Class VirtualDepthCamera

    This class implements the whole Virtual Depth Camera OpenGL Pipeline:
    + Loads the stl files
    + contructs the necessary VAOs (via the helpder class GraphicObject)
    + contructs the virtual camera

    STL filepaths are parsed automatically by the URDF, given in its compiled XML format in the constructor.
    Each robot link position in 3D space is acquired via TF_listener.

    """

    def __init__(self, mode, urdf_xml_string, ur_desc_folder_path, camera_params, width=1280, height=720, scaling=1.0):
        """
        Initialises OpenGL, laods all the graphic Objects from the collision STL file

        :param mode: "DEPTH" or "MASK".
                    - "DEPTH" generates an emulated depth image, that can be used for depth measurement of the robot geometry.
                    - "MASK" generates a simple "boolean" {0,255} mask, that can represents the presence of robot or not in the 2D image.
        :param urdf_xml_string:     The compiled XML URDF, acquired via service from robot_state_publisher.
        :param ur_desc_folder_path: The arl_descrpiton package containing the description.
                                    Replaces the "//package:" tag in the urdf filepath of mesh files.
                                    `ur_desc_folder_path = get_package_share_directory('arl_description')` would suffice
        :param camera_params:  (fx,fy,cx,cy) tuple
        :param width:    desired image width
        :param height:   desired image height
        :param scaling:  Graphic objects can be inflated by a factor of 1/scale. Better keep it between [0.98, 1.0]
        """
        self.MODE = mode

        self.width = width
        self.height = height

        self.config_dict = self.urdf_traverse(urdf_xml_string, ur_desc_folder_path, "all")

        # Create OpenGL context, via hidden glfw window
        if not glfw.init():
            raise Exception("glfw not initialized")

        # https://www.glfw.org/docs/latest/context_guide.html#context_hints
        # hide glfw window.
        glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
        self._win = glfw.create_window(self.width, self.height, "virtual depth mask window", None, None)
        if not self._win:
            glfw.terminate()
            raise Exception("glfw windows cannot be created")
        # create Opengl context, (basically init  opengl)
        glfw.make_context_current(self._win)

        # init shader program. Different shader for each mode
        if self.MODE == "DEPTH":
            shader = compileProgram(
                compileShader(gl_shaders.vertex_shader_depth_mode,   GL_VERTEX_SHADER),
                compileShader(gl_shaders.fragment_shader_depth_mode, GL_FRAGMENT_SHADER)
            )
        elif self.MODE == "MASK":
            shader = compileProgram(
                compileShader(gl_shaders.vertex_shader_mask_mode,   GL_VERTEX_SHADER),
                compileShader(gl_shaders.fragment_shader_mask_mode, GL_FRAGMENT_SHADER)
            )
        else:
            glfw.terminate()
            raise NotImplementedError

        # variable locations in the Shader Program
        # used by each Graphic:
        self.position_loc = glGetAttribLocation(shader, "a_position")

        self.model_loc  = glGetUniformLocation(shader, "view_model")
        self.origin_loc = glGetUniformLocation(shader, "origin")
        self.fix_tf_loc = glGetUniformLocation(shader, "fix_tf")
        self.projection_loc = glGetUniformLocation(shader, "projection")

        glUseProgram(shader)
        glClearColor(0.0, 0.0, 0.0, 1)
        glEnable(GL_DEPTH_TEST)
        glPixelStorei(GL_PACK_ALIGNMENT, 1)  # neede for the FBO.ReadPixels

        #  set projection matrix
        fx, fy, cx, cy = [c for c in camera_params]
        f = 10.0  # far plane
        n = 0.1  # near plane
        projection_tf = np.array([[fx / cx, 0, 0, 0],
                                  [0, fy / cy, 0, 0],
                                  [0, 0, -(f + n) / (f - n), -2 * f * n / (f - n)],
                                  [0, 0, -1, 0]]).transpose()

        glUniformMatrix4fv(self.projection_loc, 1, GL_FALSE, projection_tf)

        # correction TF that rotates camera around its z axis. IDK why, but for some reason OpenGL camera points ... backwards?
        fixed_tf = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        glUniformMatrix4fv(self.fix_tf_loc, 1, GL_FALSE, fixed_tf)

        # Construct list of Graphics, according to the config-file specifitcations
        self.graphics_object_list = []
        for k_ in list(self.config_dict.keys()):
            f_ = self.config_dict[k_]["mesh_stl"]
            origin_t_ = self.config_dict[k_]["origin"]["xyz"]
            origin_rpy_ = self.config_dict[k_]["origin"]["rpy"]
            R_ = transforms3d.euler.euler2mat(origin_rpy_[0], origin_rpy_[1], origin_rpy_[2], axes='sxyz')
            origin_tf = np.eye(4)
            origin_tf[0:3, 0:3] = R_
            origin_tf[0:3, 3] = origin_t_.ravel()
            g_ = GraphicObject(f_,
                               self.position_loc,
                               self.model_loc,
                               self.origin_loc, origin_tf,
                               scaling=scaling)
            self.graphics_object_list.append(g_)

    def get_tf_link_names(self):
        return self.config_dict.keys()

    def run(self, model_tf_list):
        # glfw.poll_events()  # Not needed, unless window is not hidden
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        for g_, m_tf_ in zip(self.graphics_object_list, model_tf_list):
            g_.draw(m_tf_)
        glfw.swap_buffers(self._win)

    def get_mask_image(self):
        img_buf = glReadPixels(0, 0, self.width, self.height, GL_LUMINANCE, GL_UNSIGNED_BYTE)
        img = np.frombuffer(img_buf, np.uint8).reshape(self.height, self.width, 1)[::-1]
        # the indexing above is copy-pasted from the internet, have no idea why it should be the way is its,
        # but without the image is returned broken, and have no idea how to fix it.
        return img

    def get_depth_image(self):
        img_buf = glReadPixels(0, 0, self.width, self.height, GL_DEPTH_COMPONENT, GL_FLOAT)
        z_b = np.frombuffer(img_buf, np.float32).reshape(self.height, self.width, 1)[::-1]

        # Vodoo magic to get from gl to reality
        z_n = 2 * z_b -1 
        zNear = 0.1
        zFar = 10
        z = 2 * zNear * zFar / ( zFar + zNear - z_n * (zFar - zNear))       

        img = z
        # the indexing above is copy-pasted from the internet, have no idea why it should be the way is its,
        # but without the image is returned broken, and have no idea how to fix it.
        return img

    def destroy_virtual_camera(self):
        # destructor
        glfw.terminate()

    def urdf_traverse(self, data, description_package_folder, desired_links="all"):
        # Traverse the URDF XML string and store the mesh filepath and origin point in a dict,
        #  link_name is key and the dict resemples ye old YAML file
        #
        #   Inputs:
        #       data: The XML styring, as parsed form the URDF xacro (ideally
        #                       retrieved directly from robot_state_publisher)
        #       description_package_folder: "get_package_share_directory('arl_description')"
        #       desired_links: If "all" (default) the method collects all collision data from urdf.
        #                       Alternatively it can accept a list of <link_name> (NOT IMPLEMENTED)
        #
        #   Output:
        #   Dict with content
        #       link_name_1:
        #          mesh_file: "/some/path/to/description/link_1.stl"
        #          origin:
        #              rpy = [0 0 0]
        #              xyz = [0 0 0]
        #       link_name_2:
        #           ...
        #
        from lxml import etree
        parser = etree.XMLParser(recover=True)
        root = etree.fromstring(data, parser=parser)

        # Collect Link Names
        link_name_list = []
        if type(desired_links) is list:
            link_name_list = desired_links
            raise NotImplementedError("This feature is not specified yet.")
        elif desired_links == "all":
            # Traverse XML and gather up all links that contain a mesh file under collision tag
            temp = root.xpath("//robot/link/collision/geometry/mesh")
            for t_ in temp:
                link_name = t_.getparent().getparent().getparent().attrib["name"]
                link_name_list.append(link_name)
        else:
            raise RuntimeError(
                "[VIRTUAL DEPTH CAMERA]: Invalid type: 'desired_links' input type. Input Arg must be a list of strings e..g ['base_link', 'shoulder_link' etc]")

        # Now that we have all link_names we can access the collision entries directly with the paths
        #
        #   '//robot/link[@name = "some_link"]/collision/geometry/mesh
        #  '//robot/link[@name = "some_link"]/collision/origin
        #
        mesh_dict = dict()
        for link_ in link_name_list:
            # link dict entry
            mesh_dict[link_] = dict()
            link_path_ = "//robot/link[@name='" + link_ + "']"

            # link/meshfile dict entry
            temp = root.xpath(link_path_ + "/collision/geometry/mesh")
            mesh_file = temp[0].attrib["filename"]
            mesh_file = mesh_file.replace("package://arl_description/", "")
            mesh_file = os.path.join(description_package_folder, mesh_file)
            mesh_dict[link_]["mesh_stl"] = mesh_file

            # link/origin dict entry
            origin_ = dict()
            temp = root.xpath(link_path_ + "/collision/origin")
            if len(temp) == 1:
                origin_["xyz"] = np.fromstring(temp[0].attrib["xyz"], dtype=float, sep=" ").reshape(3, 1)
                origin_["rpy"] = np.fromstring(temp[0].attrib["rpy"], dtype=float, sep=" ").reshape(3, 1)
            else:
                warnings.warn(
                    "[VIRTUAL DEPTH CAMERA]: WARNING: Link '" + link_ + "' has no origin defined in the urdf file. Assumiong default origin xyz=[0 0 0], rpy=[0 0 0]")
                origin_["xyz"] = np.zeros((3, 1))
                origin_["rpy"] = np.zeros((3, 1))
            mesh_dict[link_]["origin"] = origin_
            # print("----------------")
            # print(link_)
            # print(origin_["rpy"].reshape(1,3))
            # print(origin_["xyz"].reshape(1,3))
        return mesh_dict
