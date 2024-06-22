class Landmarks():
    def __init__(self):
        self.landmarkIndices = []
        self.landmarkCoords = []
        self.landmarkColors = []
        self.removedPool = []
        self.removedPoolSize = 0

        self.POOL_COORDS = [0,0,-100000]
        self.POOL_COLORS = [0,0,0]

        self.totalLandmarks = 0

    def addPoint(self, id, point_pos, color):
        if (self.removedPoolSize > 0):
            index = self.removedPool.pop()
            self.removedPoolSize -= 1
            if(id < len(self.landmarkIndices)):
                self.landmarkIndices.pop(id)
            self.landmarkIndices.insert(id, index)
            self.changeLandmarkCoords(index, point_pos)
            self.changeLandmarkColor(index, color)
        else:
            if (id < len(self.landmarkIndices)):
                self.landmarkIndices.pop(id)
            self.landmarkIndices.insert(id, self.totalLandmarks)
            self.landmarkCoords.insert(self.totalLandmarks, point_pos)
            self.landmarkColors.insert(self.totalLandmarks, color)

            self.totalLandmarks += 1

    def removeLandmark(self, id):
        try:
            index = self.landmarkIndices[id]
        except:
            index = None
        if (index is None or index < 0):
            return

        self.changeLandmarkCoords(index, self.POOL_COORDS)
        self.changeLandmarkColor(index, self.POOL_COLORS)

        self.landmarkIndices[id] = -1
        self.removedPool.append(index)
        self.removedPoolSize += 1

    def changeLandmarkCoords(self, index, point_pos):
        if(index < len(self.landmarkCoords)):
            self.landmarkCoords.pop(index)
        self.landmarkCoords.insert(index, point_pos)

    def changeLandmarkColor(self, index, color):
        if(index < len(self.landmarkColors)):
            self.landmarkColors.pop(index)
        self.landmarkColors.insert(index, color)

    def updateLandmark(self, id, point_pos, color):
        try:
            index = self.landmarkIndices[id]
        except:
            index = None
        if (index is None or index < 0):
            self.addPoint(id, point_pos, color)
        else:
            self.changeLandmarkCoords(index, point_pos)
            self.changeLandmarkColor(index, color)

    def getAllLandmarks(self):
        allLandmarks = []
        for i in range(len(self.landmarkIndices)):
            landmark = {}
            if (self.landmarkIndices[i] is None or self.landmarkIndices[i] < 0):
                continue
            try:
                landmark["id"] = i
                landmark["point_pos"] = self.landmarkCoords[self.landmarkIndices[i]]
                landmark["rgb"] = self.landmarkColors[self.landmarkIndices[i]]
                allLandmarks.append(landmark)
            except:
                print(i, self.landmarkIndices[i], flush=True)
                try:
                    print('coords', self.landmarkCoords[self.landmarkIndices[i]], flush=True)
                    print('color', self.landmarkColors[self.landmarkIndices[i]], flush=True)
                except:
                    print('')
                continue
        return allLandmarks
