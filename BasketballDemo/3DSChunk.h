//注释中数据类型
//
//全局chunk
#define RGB_FLOAT 0x0010		//头+float_red+float_green+float_blue 均从0.0-1.0
#define RGB_BYTE 0x0011		//头+byte_red+byte_green+byte_green 从0-255
#define RGB_BYTE_GAMMA 0x0012	//头+byte_red+byte_green+byte_blue 
#define RBG_FLOAT_GAMMA 0x0013	//头+float_red+float_green+float_blue
#define PERCENT_INT 0x0030		//头+word_percent		0-100
#define PERCENT_FLOAT 0x0031//头+float_percent 0.0-100.0

//3DS文件开始chunk
#define MAIN 0x4D4D		//头+子chunk大小
	#define MAIN_VERSION 0x0002		//大小等于头+dword内容
	#define EDITOR 0x3D3D		//头+子chunk
		#define ONEUNIT 0x0100	//头+float
		#define BACKGROUND_BITMAP 0x1100			//头+strName
		#define USE_BACKGROUND_BITMAP 0x1101	//头 这个的存在性决定着是否使用BACKGROUND_BITMAP
		#define BACKGROUND_COLOR 0x1200			//头+颜色块 一般是头+RGB_COLOR+RGB_GAMMA_COLOR
		#define USE_BACKGROUND_COLOR 0x1201	//头 这个块的存在决定着是否使用BACK_COLOR
		#define GRADIENT_COLOR 0x1300				//头+Gradient_position_float+RGB1+RGBG1+RGB2+RGBG2+RGB3+RGBG3 渐变颜色  
		#define USE_GRADIENT_COLOR 0x1301		//头 布尔量
		#define SHADOW_MAP_BIAS 0x1400			//头+float_Shadow_map_bias
		#define SHADOW_MAP_SIZE 0x1420			//头+word_shadow_map_size
		#define SHADOW_MAP_SAMPLE_RANGE 0x1450//头+float
		#define RAYTRACE_BIAS 0x1460					//头+float
		#define PAYTRACE_ON 0x1470					//头  布尔量 0x1460
		#define AMBIENT_GLOBAR_COLOR 0x2100				//头+RGB 

		#define FOG 0x2200
			#define FOG_BACKGROUND 0x2210
		#define USE_FOG 0x2201

		#define DISTANCE_QUEUE 0x2300
		#define DIM_BACKGROUND 0x2310
		#define USE_DISTANCE_QUEUE 0x2301
		#define LAYERED_FOG_OPTIONS 0x2302
		#define USE_LAYERED_FOG 0x2303

		#define MESH_VERSION 0x3D3E		//头+dword

		#define OBJECT_BLOCK 0x4000		//头+strName+子chunk
			#define OBJECT_HIDDEN 0x4010	//头 布尔量 是否隐藏物体
			#define OBJECT_NOT_CAST 0x4012//头 布尔量 
			#define MATTE_OBJECT 0x4013 //头 布尔量
			#define EXTERNAL_PROCESS_ON 0x4015	//头 布尔量
			#define OBJECT_NOT_RECEIVE_SHADOWS 0x4017	//头 布尔量 不接受阴影	未选中该项时表示由该材质组成的物体在场景中不会接受其他物体的投影

			#define TRI_MESH 0x4100		//头+子chunk
				#define VERTICES_LIST 0x4110	//头+wrod_numVertex+vector_vertexPos
				#define FACES_DESCRIPTION 0x4120	//头+word_numberFace+face(word_vertexindexA,word_Vertex_indexB,word_vertex_indexC,word_FaceFlag(bit0_CA_VISIBLE,bit1_BC_VISIBLE,bit2_AB_VISIBLE))+子chunk
					#define FACES_MATERIAL_LIST 0x4130	//头+str_MaterialName+word_num_Entries+word_face_material_index
					#define SMOOTHING_GROUP_LIST 0x4150//
				#define UV_COOR_LIST 0x4140	//头+word_num_vertex+float_Ucoor+float_Vcoor
				#define LOCAL_COOR_SYS 0x4160	//头+vector_X+vector_Y+vector_Z+vector_orgin_pos
				#define OBJECT_COLOR_IN_EDITOR 0x4165	//头+byte_Color

				#define EXTERNAL_PROCESS_NAME 0x4181	//
				#define EXTERNAL_PROCESS_PARAM 0x4182

			#define LIGHT 0x4600			//头+vector_Position+float_hotpoint+float_diffuse+byte_RGB+子chunk
				#define SPOTLIGHT 0x4610		//头+vector_Target+float_hosSpot+float_FallOff +子chunk
					#define SPOT_RAYTRACE 0x4627		//
					#define LIGHT_SHADOWED 0x4630
					#define SPOT_SHADOW_MAP 0x4641
					#define SPOT_SHOW_CONE 0x4650
					#define SPOT_IS_RECT 0x4651		//头  这个光源是否是方形的
					#define SPOT_OVERSHOOT 0x4652
					#define SPOT_MAP 0x4653			//头+str_FileName
					#define SPOT_ROLL 0x4656			//头+float_Roll(是一个角度)
					#define SPOT_RAYTRACE_BIAS 0x4658	

				#define LIGHT_OFF 0x4620
				#define ATTENUATION_ON 0x4625
				#define RANGE_START 0x4659
				#define RANGE_END 0x465A
				#define MULTIPLIER 0x465B

			#define CAMERA 0x4700		//头+vector_Pos+vector_Target+float_Bank(degree)+float_Length
		#define WINDOW_SET 0x7001
			#define WINDOW_DESCRIP2 0x7011
			#define WINDOW_DESCRIP1 0x7012
			#define MESH_WINDOWS 0x7020

		#define MATERIAL 0xafff		//头+子chunk
			#define MATERIAL_NAME 0xA000	//头+str_MaterialName
			#define AMBIENT_COLOR 0xA010	//0+子chunk 其中还有RGB_BYTE和RGB_BYTE_GAMMA
			#define DIFFUSE_COLOR 0xa020		//0+子chunk 其中还有RGB_BYTE和RGB_BYTE_GAMMA
			#define SPECULAR_COLOR 0xA030//0+子chunk 其中还有RGB_BYTE和RGB_BYTE_GAMMA

			#define SHININESS_PERCENT 0xA040//0+子chunk	其中还有PERCENT_INT 
			#define SHININESS_STRENGTH_PERCENT 0xA041//0+子chunk	其中还有PERCENT_INT 
			#define TRANSPARENCY_PERCENT 0xA050	//透明度
			#define TRANSPARENCY_FALLOFF_PERCENT 0xA052
			#define REFLECTION_BLUR_PERCENT 0xA053

			#define TWO_SIDED 0xA081
			#define ADD_TRANS 0xA083
			#define SELF_ILLUM 0xA084
			#define WIRE_FRAME_ON 0xA085
			#define WIRE_THICKNESS 0xA087
			#define FACE_MAP 0xA088
			#define IN_TRANC 0xA08A
			#define SOFTEN 0xA08C
			#define WIRE_IN_UNITS 0xA08E

			#define RENDER_TYPE 0xA100
			#define TRANSPARENCY_FALLOFF_PERCENT_PRESENT 0xA240
			#define REFLECTION_BLUR_PERCENT_PRESENT 0xA250
			#define BUMP_MAP_PRESENT 0xA252
			//下面从0xA200-0xA34C 长度均为 头+子chunk  其中子chunk是0xA300-0xA368
			#define TEXTURE_MAP_1 0xA200
			#define TEXTURE_MAP_2 0xA33A
			#define OPACITY_MAP 0xA210
			#define BUMP_MAP 0xA230
			#define SHININESS_MAP 0xA33C
			#define SPECULAR_MAP 0xA204
			#define SELF_ILLUM_MAP 0xA33D
			#define REFLECTION_MAP 0xA220
			#define MASK_FOR_TEXTURE_MAP_1 0xA33E
			#define MASK_FOR_TEXTURE_MAP_2 0xA340
			#define MASK_FOR_OPACITY_MAP 0xA342
			#define MASK_FOR_BUMP_MAP 0xA344
			#define MASK_FOR_SHININESS_MAP 0xA346
			#define MASK_FOR_SPECULAR_MAP 0xA348
			#define MASK_FOR_SELF_ILLUM_MAP 0xA34A
			#define MASK_FOR_REFLECTION_MAP 0xA34C
			//子chunk  A200—A34C
				#define MAP_FILE_NAME 0xA300		//头+str_Filename
				#define MAP_PARAM 0xA351			//
				#define BLUR_PERCENT 0xA353
				#define V_SCALE 0xA354			//头+float_VSCALE
				#define U_SCALE 0xA356			//头+float_USCALE
				#define U_OFFSET 0xA358		//头+float_UOFFSET
				#define V_OFFSET 0xA35A		//头+float_VOffset
				#define ROTATION_ANGLE 0xA35C	//头+float_RotationAngle
				#define RGB_ALPHA_TINT_1 0xA360
				#define RGB_ALPHA_TINT_2 0xA362
				#define RGB_TINT_R 0xA364
				#define RGB_TINT_G 0xA366
				#define RGB_TINT_B 0xA368


	#define KEYFRAME 0xB000			//头+子chunk
		#define AMBIENT_LIGHT_INFORMATION 0xB001
		#define MESH_INFORMATION 0xB002
		#define CAMERA_INFORMATION 0xB003
		#define CAMERA_TARGET_INFORMATION 0xB004
		#define OMNI_LIGHT_INFORMATION 0xB005
		#define SPOT_LIGHT_TARGET_INFORMATION 0xB006
		#define SPOT_LIGHT_INFORMATION 0xB007
		#define FRAMES 0xB008	//头+dword_Start+dword_End
//下面这些都是0xB001-0xB007的子chunk
			#define OBJECT_NAME 0xB010	//头+str_ObjName+word_Flag1_Bit11_Hidden+word_Flag2( show path 0,animate smoothing 1,object motion blur 4,morph materials 6)+word_Hierarchy_father(link to the parent object -1为NULL)
			#define OBJECT_PIVOT_POINT 0xB013//头+vector_pivot_point 物体轴心点
			#define OBJECT_MORPH_ANGLE 0xB015//头+float_morph_smoothing_angle_rad  

/*Chunk #    : 0xB020..0xB029
Name       : Track
Level      : 3
Size       : varying
Father     : 0xB001..0xB007 Information block
Format     :

  word     Flag
            * Bits 0-1 : 0 = single
                         2 = repeat
                         3 = loop

            * Bit 3 : lock X
            * Bit 4 : lock Y
            * Bit 5 : lock Z

            * Bit 7 : unlink X
            * Bit 8 : unlink Y
            * Bit 9 : unlink Z

  8 byte   Unknown
  dword    Number of keys in this track

           Then, for each key:
		   {
  dword    Key number (position in track)
  word     Acceleration data present (flag)   Range:
            * Bit 0 : Tension follows         [-1.0, 1.0]
            * Bit 1 : Continuity follows      [-1.0, 1.0]
            * Bit 2 : Bias follows            [-1.0, 1.0]
            * Bit 3 : Ease to follows         [ 0.0, 1.0]
            * Bit 4 : Ease from follows       [ 0.0, 1.0]
   float Acceleration data
          Track specific data
		  }

  Track specific data is:
   0xB020 : Position track : 1 vector   Position
   0xB021 : Rotation track : 1 float    Angle (rad)
                             1 vector   Axis
   0xB022 : Scale track    : 3 floats   Size
   0xB023 : FOV track      : 1 float    Angle (degree)
   0xB024 : Roll track     : 1 float    Angle (degree)
   0xB025 : Color track    :
   0xB026 : Morph track    : 1 strz     Object name
   0xB027 : Hotspot track  : 1 float    Angle (degree)
   0xB028 : Falloff track  : 1 float    Angle (degree)
   0xB029 : Hide track     : nothing
*/

			#define POSITION_TRACK 0xB020
			#define RATATION_TRACK 0xB021
			#define SCALE_TRACK 0xB022
			#define FOV_TRACK 0xB023
			#define ROLL_TRACK 0xB024
			#define COLOR_TRACK 0xB025
			#define MORPH_TRACK 0xB026
			#define HOTSPOT_TRACK 0xB027
			#define FALLOFF_TRACK 0xB028
			#define HIDE_TRACK 0xB029

			#define HIERARCHY_POSITION 0xB030//头+word_hierarchy

