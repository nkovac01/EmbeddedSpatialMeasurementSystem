import numpy as np
import open3d as o3d

pcd = o3d.io.read_point_cloud('tof_radar.xyz', format='xyz')
#print(pcd)

#print(np.asarray(pcd.points))
# o3d.visualization.draw_geometries([pcd])        # visualize the point cloud

pt1=0
pt2=1
pt3=2
pt4=3
pt5=4
pt6=5
pt7=6
pt8=7
pt9=8
pt10=9
pt11=10
pt12=11
pt13=12
pt14=13
pt15=14
pt16=15
pt17=16
pt18=17
pt19=18
pt20=19
pt21=20
pt22=21
pt23=22
pt24=23
pt25=24
pt26=25
pt27=26
pt28=27
pt29=28
pt30=29
pt31=30
pt32=31
po = 0

lines = []                                      # empty array for a line set

# connect all points within a plane
for i in range(10):
    lines.append([pt1+po,pt2+po])
    lines.append([pt2+po,pt3+po])
    lines.append([pt3+po,pt4+po])
    lines.append([pt4+po,pt5+po])
    lines.append([pt5+po,pt6+po])
    lines.append([pt6+po,pt7+po])
    lines.append([pt7+po,pt8+po])
    lines.append([pt8+po,pt9+po])
    lines.append([pt9+po,pt10+po])
    lines.append([pt10+po,pt11+po])
    lines.append([pt11+po,pt12+po])
    lines.append([pt12+po,pt13+po])
    lines.append([pt13+po,pt14+po])
    lines.append([pt14+po,pt15+po])
    lines.append([pt15+po,pt16+po])
    lines.append([pt16+po,pt17+po])
    lines.append([pt17+po,pt18+po])
    lines.append([pt18+po,pt19+po])
    lines.append([pt19+po,pt20+po])
    lines.append([pt20+po,pt21+po])
    lines.append([pt21+po,pt22+po])
    lines.append([pt22+po,pt23+po])
    lines.append([pt23+po,pt24+po])
    lines.append([pt24+po,pt25+po])
    lines.append([pt25+po,pt26+po])
    lines.append([pt26+po,pt27+po])
    lines.append([pt27+po,pt28+po])
    lines.append([pt28+po,pt29+po])
    lines.append([pt29+po,pt30+po])
    lines.append([pt30+po,pt31+po])
    lines.append([pt31+po,pt32+po])
    lines.append([pt32+po,pt1+po])
    po+=32

pt1=0
pt2=1
pt3=2
pt4=3
pt5=4
pt6=5
pt7=6
pt8=7
pt9=8
pt10=9
pt11=10
pt12=11
pt13=12
pt14=13
pt15=14
pt16=15
pt17=16
pt18=17
pt19=18
pt20=19
pt21=20
pt22=21
pt23=22
pt24=23
pt25=24
pt26=25
pt27=26
pt28=27
pt29=28
pt30=29
pt31=30
pt32=31
po = 0
do = 32

# connect vertices between planes
for i in range(9):
    lines.append([pt1+po,pt1+po+do])
    lines.append([pt2+po,pt2+po+do])
    lines.append([pt3+po,pt3+po+do])
    lines.append([pt4+po,pt4+po+do])
    lines.append([pt5+po,pt5+po+do])
    lines.append([pt6+po,pt6+po+do])
    lines.append([pt7+po,pt7+po+do])
    lines.append([pt8+po,pt8+po+do])
    lines.append([pt9+po,pt9+po+do])
    lines.append([pt10+po,pt10+po+do])
    lines.append([pt11+po,pt11+po+do])
    lines.append([pt12+po,pt12+po+do])
    lines.append([pt13+po,pt13+po+do])
    lines.append([pt14+po,pt14+po+do])
    lines.append([pt15+po,pt15+po+do])
    lines.append([pt16+po,pt16+po+do])
    lines.append([pt17+po,pt17+po+do])
    lines.append([pt18+po,pt18+po+do])
    lines.append([pt19+po,pt19+po+do])
    lines.append([pt20+po,pt20+po+do])
    lines.append([pt21+po,pt21+po+do])
    lines.append([pt22+po,pt22+po+do])
    lines.append([pt23+po,pt23+po+do])
    lines.append([pt24+po,pt24+po+do])
    lines.append([pt25+po,pt25+po+do])
    lines.append([pt26+po,pt26+po+do])
    lines.append([pt27+po,pt27+po+do])
    lines.append([pt28+po,pt28+po+do])
    lines.append([pt29+po,pt29+po+do])
    lines.append([pt30+po,pt30+po+do])
    lines.append([pt31+po,pt31+po+do])
    lines.append([pt32+po,pt32+po+do])
    po+=32

line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)), lines = o3d.utility.Vector2iVector(lines))
o3d.visualization.draw_geometries([line_set])
