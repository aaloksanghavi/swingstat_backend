import math
import json

def radians_to_degrees(radians):
    return radians * (180/math.pi)

# Compares left arm angle at top of backswing to threshold
def detectBentLeftArm(poseLandmarks, threshold, keyFrame):
    leftShoulderX = poseLandmarks['poses'][str(keyFrame)]['poseLandmarks'][str(11)]['position']['x']
    leftShoulderY = poseLandmarks['poses'][str(keyFrame)]['poseLandmarks'][str(11)]['position']['y']
    leftElbowX = poseLandmarks['poses'][str(keyFrame)]['poseLandmarks'][str(13)]['position']['x']
    leftElbowY = poseLandmarks['poses'][str(keyFrame)]['poseLandmarks'][str(13)]['position']['y']
    leftWristX = poseLandmarks['poses'][str(keyFrame)]['poseLandmarks'][str(15)]['position']['x']
    leftWristY = poseLandmarks['poses'][str(keyFrame)]['poseLandmarks'][str(15)]['position']['y']
    
    leftElbowToShoulderYDiff = leftElbowY - leftShoulderY
    leftShoulderToElbowXDiff = leftShoulderX - leftElbowX
    leftElbowToWristYDiff = leftElbowY - leftWristY
    leftElbowToWristXDiff = leftWristX - leftElbowX
    
    leftArmAngleRads = math.pi - abs(math.atan(leftElbowToWristYDiff / leftElbowToWristXDiff))
    leftArmAngleDegrees = radians_to_degrees(leftArmAngleRads) - radians_to_degrees(math.atan(leftElbowToShoulderYDiff / leftShoulderToElbowXDiff))
    
#     return leftArmAngleDegrees
    
    
    if leftArmAngleDegrees >= threshold:
        return True
    else:
        return False
    
# Compares total lateral hip movement to threshold
def detectHipSway(poseLandmarks, threshold, setupFrame, backswingFrame, impactFrame):
    totalLateralMovement = 0
    
#     totalLateralMovement = abs(poseLandmarks['poses'][str(setupFrame)]['poseLandmarks'][str(24)]['position']['x'] - poseLandmarks['poses'][str(backswingFrame)]['poseLandmarks'][str(24)]['position']['x'])
    
    mid_1 = (backswingFrame - setupFrame) // 2
    
    totalLateralMovement = abs(poseLandmarks['poses'][str(setupFrame)]['poseLandmarks'][str(24)]['position']['x'] - poseLandmarks['poses'][str(mid_1)]['poseLandmarks'][str(24)]['position']['x'])
    totalLateralMovement += abs(poseLandmarks['poses'][str(mid_1)]['poseLandmarks'][str(24)]['position']['x'] - poseLandmarks['poses'][str(backswingFrame)]['poseLandmarks'][str(24)]['position']['x'])

    
#     for frame in range(setupFrame, backswingFrame+1):
#         totalLateralMovement += abs(poseLandmarks['poses'][str(frame-1)]['poseLandmarks'][str(24)]['position']['x'] - poseLandmarks['poses'][str(frame)]['poseLandmarks'][str(24)]['position']['x'])
    
#     return totalLateralMovement
    if totalLateralMovement <= threshold:
        return True
    else:
        return False
    
# Compares total lateral nose movement to threshold
def detectLateralHeadMovement(poseLandmarks, threshold, setupFrame, backswingFrame, impactFrame):
    
    mid_1 = (backswingFrame - setupFrame) // 2
    mid_2 = (impactFrame - backswingFrame) // 2
    
    totalLateralMovement = abs(poseLandmarks['poses'][str(setupFrame)]['poseLandmarks'][str(0)]['position']['x'] - poseLandmarks['poses'][str(mid_1)]['poseLandmarks'][str(0)]['position']['x'])
    totalLateralMovement += abs(poseLandmarks['poses'][str(mid_1)]['poseLandmarks'][str(0)]['position']['x'] - poseLandmarks['poses'][str(backswingFrame)]['poseLandmarks'][str(0)]['position']['x'])
    totalLateralMovement += abs(poseLandmarks['poses'][str(backswingFrame)]['poseLandmarks'][str(0)]['position']['x'] - poseLandmarks['poses'][str(mid_2)]['poseLandmarks'][str(0)]['position']['x'])
    totalLateralMovement += abs(poseLandmarks['poses'][str(mid_2)]['poseLandmarks'][str(0)]['position']['x'] - poseLandmarks['poses'][str(impactFrame)]['poseLandmarks'][str(0)]['position']['x'])    
    
#     totalLateralMovement = 0
#     for frame in range(setupFrame,impactFrame+1):
#         totalLateralMovement += abs(poseLandmarks['poses'][str(frame-1)]['poseLandmarks'][str(0)]['position']['x'] - poseLandmarks['poses'][str(frame)]['poseLandmarks'][str(0)]['position']['x'])
    
#     return totalLateralMovement
    
    if totalLateralMovement <= threshold:
        return True
    else:
        return False

# Compares total vertical nose movement to threshold
def detectVerticalHeadMovement(poseLandmarks, threshold, setupFrame, backswingFrame, impactFrame):
    
    mid_1 = (backswingFrame - setupFrame) // 2
    mid_2 = (impactFrame - backswingFrame) // 2
    
    totalVerticalMovement = abs(poseLandmarks['poses'][str(setupFrame)]['poseLandmarks'][str(0)]['position']['y'] - poseLandmarks['poses'][str(mid_1)]['poseLandmarks'][str(0)]['position']['y'])
    totalVerticalMovement += abs(poseLandmarks['poses'][str(mid_1)]['poseLandmarks'][str(0)]['position']['y'] - poseLandmarks['poses'][str(backswingFrame)]['poseLandmarks'][str(0)]['position']['y'])
    totalVerticalMovement += abs(poseLandmarks['poses'][str(backswingFrame)]['poseLandmarks'][str(0)]['position']['y'] - poseLandmarks['poses'][str(mid_2)]['poseLandmarks'][str(0)]['position']['y'])
    totalVerticalMovement += abs(poseLandmarks['poses'][str(mid_2)]['poseLandmarks'][str(0)]['position']['y'] - poseLandmarks['poses'][str(impactFrame)]['poseLandmarks'][str(0)]['position']['y'])    
    
#     totalVerticalMovement = abs(poseLandmarks['poses'][str(setupFrame)]['poseLandmarks'][str(0)]['position']['y'] - poseLandmarks['poses'][str(backswingFrame)]['poseLandmarks'][str(0)]['position']['y'])
#     totalVerticalMovement += abs(poseLandmarks['poses'][str(backswingFrame)]['poseLandmarks'][str(0)]['position']['y'] - poseLandmarks['poses'][str(impactFrame)]['poseLandmarks'][str(0)]['position']['y'])

#     totalVerticalMovement = 0
#     for frame in range(setupFrame,impactFrame+1):
#         totalVerticalMovement += abs(poseLandmarks['poses'][str(frame-1)]['poseLandmarks'][str(0)]['position']['y'] - poseLandmarks['poses'][str(frame)]['poseLandmarks'][str(0)]['position']['y'])
    
#     return totalVerticalMovement

    if totalVerticalMovement <= threshold:
        return True
    else:
        return False

# Compares average excess (over 90 degrees) right wrist angle to threshold
def detectClubHeadLag(poseLandmarks, threshold, setupFrame, backswingFrame, impactFrame):
#     totalExcessWristAngle = 0
#     totalFrames = poseLandmarks['impactFrame'] - poseLandmarks['setupFrame']
    
#     for frame in range(setupFrame, impactFrame+1):
    rightWristtoThumbYDiff = abs(poseLandmarks['poses'][str(backswingFrame)]['poseLandmarks'][str(22)]['position']['y'] - poseLandmarks['poses'][str(backswingFrame)]['poseLandmarks'][str(16)]['position']['y'])
    rightWristtoThumbXDiff = abs(poseLandmarks['poses'][str(backswingFrame)]['poseLandmarks'][str(22)]['position']['x'] - poseLandmarks['poses'][str(backswingFrame)]['poseLandmarks'][str(16)]['position']['x'])
    rightWristtoThumbAngle = radians_to_degrees(math.atan(rightWristtoThumbXDiff // rightWristtoThumbYDiff))
        
    rightElbowtoWristXDiff = abs(poseLandmarks['poses'][str(backswingFrame)]['poseLandmarks'][str(16)]['position']['x'] - poseLandmarks['poses'][str(backswingFrame)]['poseLandmarks'][str(14)]['position']['x'])
    rightElbowtoWristYDiff = abs(poseLandmarks['poses'][str(backswingFrame)]['poseLandmarks'][str(16)]['position']['y'] - poseLandmarks['poses'][str(backswingFrame)]['poseLandmarks'][str(14)]['position']['y'])
    rightElbowtoWristAngle = radians_to_degrees(math.atan(rightElbowtoWristYDiff // rightElbowtoWristXDiff))

    totalWristAngle = rightWristtoThumbAngle + 90 + rightElbowtoWristAngle

#     averageExcessWristAngleDegrees = radians_to_degrees(totalExcessWristAngle) // totalFrames

    return totalWristAngle
    
#     if averageExcessWristAngleDegrees <= threshold:
#         return True
#     else:
#         return False

# Compares average right elbow angle to a threshold range (should be approximately 90 degrees from setup to impact)
def detectRightElbowAngle(poseLandmarks, thresholdLow, thresholdHigh, lastKeyFrame):
    totalRightElbowAngle = 0
    totalFrames = lastKeyFrame
    
    for frame in range(lastKeyFrame):
        rightElbowToShoulderXDiff = abs(poseLandmarks[frame][12]['x'] - poseLandmarks[frame][14]['x'])
        rightElbowToShoulderYDiff = abs(poseLandmarks[frame][12]['y'] - poseLandmarks[frame][14]['y'])
        angle_1 = math.atan(rightElbowToShoulderXDiff // rightElbowToShoulderYDiff)
        
        rightWristToElbowXDiff = abs(poseLandmarks[frame][14]['x'] - poseLandmarks[frame][16]['x'])
        rightWristToElbowYDiff = abs(poseLandmarks[frame][14]['y'] - poseLandmarks[frame][16]['y'])
        angle_2 = math.atan(rightWristToElbowXDiff // rightWristToElbowYDiff)
        
        totalRightElbowAngle += angle_1 + angle_2
    
    averageRightElbowAngleDegrees = radians_to_degrees(totalRightElbowAngle) // totalFrames
    
    if thresholdLow <= averageRightElbowAngleDegrees <= thresholdHigh:
        return True
    else:
        return False

def analyze_swing(data):

    result = {
        "left arm angle": detectBentLeftArm(data, 170, data['backswingFrame']),
        "lateral head movement": detectLateralHeadMovement(data, 80, data['setupFrame'], data['backswingFrame'], data['impactFrame']),
        "vertical head movement": detectVerticalHeadMovement(data, 90, data['setupFrame'], data['backswingFrame'], data['impactFrame']),
        "hip sway": detectHipSway(data, 40, data['setupFrame'], data['backswingFrame'], data['impactFrame'])
    }

    return result

    # print("is left arm good?")
    # print(detectBentLeftArm(data, 170, data['backswingFrame']))
    # print("is lateral head movement good?") # largely working
    # print(detectLateralHeadMovement(data, 80, data['setupFrame'], data['backswingFrame'], data['impactFrame']))
    # print("is vertical head movement good?") # largely working
    # print(detectVerticalHeadMovement(data, 90, data['setupFrame'], data['backswingFrame'], data['impactFrame']))
    # print("is hip sway good?") # largely working
    # print(detectHipSway(data, 40, data['setupFrame'], data['backswingFrame'], data['impactFrame']))

    # f.close()

#print(analyze_swing('goodconnorswing_2.json'))