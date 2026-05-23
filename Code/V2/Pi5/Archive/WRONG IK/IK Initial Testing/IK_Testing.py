import math

x = 2
y = 8
z = 10

a = 5
b = 9       
c = 9

def radian_to_degree(radians):
    return (180/math.pi)*radians

hip_rotation = math.atan2(x, z)+math.acos(a/(math.sqrt(x**2+z**2)))

hip_pitch = math.atan2(y, z) + math.acos((b**2+((z-(a*math.cos(hip_rotation)))**2+y**2)-c**2)/(2*b*math.sqrt(z-(a*math.cos(hip_rotation))**2+y**2)))

knee = math.acos((b**2+c**2-((z-(a*math.cos(hip_rotation)))**2+y**2))/(2*b*c))

hip_rotation = radian_to_degree(hip_rotation)
hip_pitch = radian_to_degree(hip_pitch)
knee = radian_to_degree(knee)

print(f"Hip Rotation Angle: {hip_rotation}")
print(f"Hip Pitch Angle: {hip_pitch}")
print(f"Knee Ange: {knee}")