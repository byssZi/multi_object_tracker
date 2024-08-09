std_msgs/Header                 header

int32 id

# verbatim (language="comment",  text="障碍物中心位置")
geometry_msgs/Point position

geometry_msgs/Quaternion orientation2

# verbatim (language="comment",  text="obstacle type: x,y,z axistrans ")
geometry_msgs/Point velocity

float64 length

float64 width

float64 height

float64 heading

float64 angle

# verbatim (language="comment",  text="障碍物顶点数组")
geometry_msgs/Polygon polygon

# verbatim (language="comment",  text="obstacle type: UNKNOW:0, UNKNOW_MOVABLE:1, UNKNOW_UNMOVABLE:2, PEDESTRAIN:3, BICYCLE:4, VEHICLE:5 ")

uint8  type

int32 history_pointnum
float64[] History_pointX
float64[] History_pointY
float64[] History_time
float64[] History_length
float64[] History_width
