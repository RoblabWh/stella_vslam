class Transmission:
    def __init__(self, callback):
        self.msgData = ""
        self.transmissionInProgress = False
        self.finishedCallback = callback
        self.text_string_max = ""
        self.num = 0
        self.text_msgs = 10

    def receive(self, msg):
        if self.transmissionInProgress:
            if msg == "FINISH_TRANSMISSION":
                self.transmissionInProgress = False
                self.finishedCallback(self.msgData)
                if (self.num < self.text_msgs):
                    self.text_string_max += self.msgData
                    self.num += 1
                else:
                    self.num = 0
                    print(len(self.text_string_max))
                self.text_string_max += self.msgData
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
