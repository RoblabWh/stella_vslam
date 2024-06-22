class Keyframes:
    def __init__(self):
        self.keyframeIndices = []
        self.keyframePoses = []
        self.removedPool = []
        self.removedPoolSize = 0

        self.POOL_KEYFRAME_POSE = [1, 0, 0, 0, 0, 1, 0, -100000, 0, 0, 1, 0]

        self.totalFrameCount = 0
        self.numValidKeyframe = 0

    def addKeyframe(self, id, pose):
        if (self.removedPoolSize > 0):
            index = self.removedPool.pop()
            self.removedPoolSize -= 1
            if(id < len(self.keyframeIndices)):
                self.keyframeIndices.pop(id)
            self.keyframeIndices.insert(id,index)
            self.changeKeyframePos(index, pose)
        else:
            if (id < len(self.keyframeIndices)):
                self.keyframeIndices.pop(id)
            self.keyframeIndices.insert(id,self.totalFrameCount)
            self.keyframePoses.insert(self.totalFrameCount, pose)

            self.totalFrameCount += 1

        self.numValidKeyframe += 1

    def removeKeyframe(self, id):
        try:
            index = self.keyframeIndices[id]
        except:
            index = None
        if(self.keyframeIndices[id] < 0 or index is None):
            return

        self.changeKeyframePos(index, self.POOL_KEYFRAME_POSE)

        self.keyframeIndices[id] = -1
        self.removedPool.append(index)
        self.removedPoolSize += 1
        self.numValidKeyframe -= 1

    def changeKeyframePos(self, index, pose):
        if (index < len(self.keyframePoses)):
            self.keyframePoses.pop(index)
        self.keyframePoses.insert(index,pose)

    def updateKeyframe(self, id, pose):
        try:
            index = self.keyframeIndices[id]
        except:
            index = None
        if (index is None or index < 0):
            self.addKeyframe(id, pose)
        else:
            self.changeKeyframePos(index, pose)

    def getAllKeyframes(self):
        allKeyframes = []
        for i in range(len(self.keyframeIndices)):
            keyframe = {}
            if (self.keyframeIndices[i] is None or self.keyframeIndices[i] < 0):
                continue
            keyframe["id"] = i
            keyframe["pose"] = self.keyframePoses[self.keyframeIndices[i]]
            allKeyframes.append(keyframe)
        return allKeyframes
