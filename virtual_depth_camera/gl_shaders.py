vertex_shader_mask_mode = """
       #version 330
        
        uniform mat4 projection;
        uniform mat4 fix_tf;
        uniform mat4 view_model;
        uniform mat4 origin;

        in vec3 a_position; // vertex coordinates from STL

        out vec3 v_color;

        void main()
        {
            // gl_Position is a special variable that holds the position of the vertex in clip space.
            // vertex coordinates projected to pinhole camera 
            gl_Position = projection *  fix_tf * view_model * origin * vec4(a_position, 1.0); 
            v_color = vec3(1.0, 1.0, 1.0); 
        }
        """

fragment_shader_mask_mode = """
        #version 330

        in vec3 v_color;
        out vec4 out_color;

        void main()
        {
            out_color = vec4(v_color, 1.0);
        }
        """

vertex_shader_depth_mode = """
       #version 330
        
        uniform mat4 view_model;
        uniform mat4 fix_tf;
        uniform mat4 projection;
        uniform mat4 origin;

        in vec3 a_position; // vertex coordinates from STL
        vec4 temp;
        
        out vec3 v_color;
        
        void main()
        {
            // gl_Position is a special variable that holds the position of the vertex in clip space.
            // vertex coordinates projected to pinhole camera 
            temp =  view_model * origin * vec4(a_position, 1.0); // vertex coordinates wrt to the camera frame
            gl_Position =  projection * fix_tf * temp; 
            
            v_color = vec3(1.0, 1.0, 1.0);
            v_color = vec3(1.0, 1.0, 1.0); //*(temp[2]/3.0/2.0);
        }
        """

fragment_shader_depth_mode = """
        #version 330

        in vec3 v_color;
        out vec4 out_color;

        void main()
        {
            //float ndcDepth =  (2.0 * gl_FragCoord.z - gl_DepthRange.near - gl_DepthRange.far) / (gl_DepthRange.far - gl_DepthRange.near);
            //float ndcDepth =  (2.0 * gl_FragCoord.z - 0.1 - 3.0) / (0.1 - 3.0);
            //float clipDepth = ndcDepth / 3.0;

            //out_color = vec4((clipDepth * 0.5) + 0.5);
            out_color = vec4(v_color, 1.0);
        }
        """

