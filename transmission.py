class Transmission:
    def __init__(self, callback):
        self.msgData = ""
        self.transmissionInProgress = False
        self.finishedCallback = callback

    def receive(self, msg):
        if self.transmissionInProgress:
            if msg == "FINISH_TRANSMISSION":
                self.transmissionInProgress = False
                self.finishedCallback(self.msgData)
                self.msgData = ""
            else:
                self.msgData += msg
        else:
            if msg == "START_TRANSMISSION":
                self.transmissionInProgress = True
            else:
                self.transmissionInProgress = False
                self.msgData = ""
                #print("Transmission state missmatch. Resetting transmission, some data might be lost.")
