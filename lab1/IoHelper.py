import os
from panda3d.core import Vec3
from panda3d.core import Vec2
import re

class IoHelper:
    def __init__(self, filePath):
        self.mFilePath = filePath
        self.mCurCharIdx = 0;
        self.mCurLineIdx = 1

    def open(self):
        self.fileHandler = open(self.mFilePath,"r")
    
    def close(self):
        self.fileHandler.close();

    def getCurLineAndChar(self):
        return [self.mCurLineIdx,self.mCurCharIdx]

    def readNextToken(self):
        strRet = ""
        nxtChar = ' '
        while nxtChar ==' ' or nxtChar == '\n' or nxtChar == '\r':
            nxtChar = self.fileHandler.read(1)
            self.mCurCharIdx  = self.mCurCharIdx + 1
            if nxtChar == '\n' :
                self.mCurLineIdx = self.mCurLineIdx + 1;
                self.mCurCharIdx = 0


        while nxtChar != ' ' and nxtChar != '\n':
            strRet = strRet + nxtChar
            nxtChar = self.fileHandler.read(1)
            if not nxtChar :
                if len(strRet) > 0 :
                    return strRet
                else:
                    return
            self.mCurCharIdx  = self.mCurCharIdx + 1

        if nxtChar == '\n' :
            self.mCurLineIdx = self.mCurLineIdx + 1;
            self.mCurCharIdx = 0
        else:
            self.mCurCharIdx  = self.mCurCharIdx + 1

        return strRet

class ParseContext:
    def __init__(self):
        self.scope = 0
        self.curToken = ""

    def addScopeDep(self):
        self.scope = self.scope + 1
    
    def decScopeDep(self):
        self.scope = self.scope - 1
        assert self.scope >=0 , "Bad scope number"

    def getCurScopeDep(self):
        return self.scope
    
    def setCurToken(self,token):
        self.curToken = token

    def getCurToken(self):
        return self.curToken
    
class BvhReader:
    def __init__(self,bvhPath):
        self.mFileReader = IoHelper(bvhPath)
        self.mFileReader.open()
        self.mContext = ParseContext()
        self.mRootName = []
        self.mRootOffset = []
        self.mRootChannels = []
        self.mRootJointsIdx = []
        self.mJointNames = []
        self.mOffsets = []
        self.mJointParents = []
        self.mRootStack = []
        self.mJointStack = []
        self.mJointChannels = []
        self.mJointFrameMotionData = []
        self.mFrameNum = 0
        self.mFrameTime = 0
    
    def errorString(self,notErrorCondition,errorWhy):
        if notErrorCondition :
            return

        errorPlace = self.mFileReader.getCurLineAndChar()
        print( "Parse Error in line" +errorPlace[0] + " char "+errorPlace[1] + ", Hint: " + errorWhy)
        raise ValueError("parse error")

    def is_number(self,s):
        # 匹配整数或浮点数（包括可选的正负号）
        pattern = r'^[-+]?\d*\.?\d+$'
        return bool(re.match(pattern, s))

    def cur(self):
        return self.mContext.getCurToken()

    def next(self):
        nxtCh = self.mFileReader.readNextToken()

        if not nxtCh:
            self.errorString(False,"Unexpected file early EOF")
            return
        self.mContext.setCurToken(nxtCh)

        if self.cur() == '{':
            self.addScope()
        if self.cur() == '}':
            self.decScope()

        if __name__ == "__main__":
            print(self.cur()+" ")

        return self.cur()

    def addScope(self):
        self.mContext.addScopeDep()
        return self.mContext.getCurScopeDep()
    
    def decScope(self):
        self.mContext.decScopeDep()
        return self.mContext.getCurScopeDep()

    def __del__(self):
        self.mFileReader.close()

    def parseHierarchy(self):
        self.errorString( self.cur() == "HIERARCHY" , "Bad Parse")
        self.next()
        self.mJointStack.append(-1)
        while self.cur() == "ROOT":
            self.parseRoot()
        self.mJointStack.pop()
            
    def parseRoot(self):
        self.errorString( self.cur() == "ROOT" , "Bad Parse")
        self.mRootStack.append(self.next()) ## RootName inside
        self.mRootName.append(self.cur())

        ## 注意需要包含根节点和末端，根节点起名为RootJoint，末端起名为父节点+'_end'
        self.mJointParents.append(self.mJointStack[-1])
        self.mJointStack.append(len(self.mJointNames))
        self.mJointNames.append(self.cur())

        self.mRootJointsIdx.append(len(self.mJointNames))
        self.errorString( self.next() == "{" , "Bad Parse")
        self.next()
        self.mRootOffset.append(self.parseOffset())
        self.mOffsets.append(self.mRootOffset[-1])
        self.next()
        self.mRootChannels.append(self.parseChannels())
        self.mJointChannels.append(self.mRootChannels[-1])
        self.next()

        while self.cur() == "JOINT":
            self.parseJoint()
            self.next()
        self.mJointStack.pop()
        self.mRootStack.pop() ## RootName inside

    def parseChannels(self):
        self.errorString(self.cur() == "CHANNELS" , "Bad Parse")
        self.errorString(self.is_number(self.next()) , "Bad Parse")
        channelNum = int(self.cur())
        channels = []
        for i in range(0,channelNum) :
            self.next()
            self.errorString(self.cur() == "Xposition" or self.cur() == "Yposition" or self.cur() == "Zposition" or self.cur() == "Xrotation" or self.cur() == "Yrotation" or self.cur() == "Zrotation", "Bad channel")
            channels.append(self.cur())
        return channels

    def parseOffset(self):
        self.errorString(self.cur() == "OFFSET" , "Bad Parse")
        offsetX = self.next()
        self.errorString(self.is_number(offsetX) == True , "Bad Offset")
        offsetY = self.next()
        self.errorString( self.is_number(offsetY) == True , "Bad Offset")
        offsetZ = self.next()
        self.errorString( self.is_number(offsetZ) == True , "Bad Offset")
        offsetVec = Vec3(float(offsetX),float(offsetY),float(offsetZ))
        return offsetVec

    def parseJoint(self):
        self.errorString( self.cur() == "JOINT" , "Bad Parse")
        self.next() 
        self.mJointParents.append(self.mJointStack[-1])
        self.mJointStack.append(len(self.mJointNames))
        self.mJointNames.append(self.cur())
        self.errorString( self.next() == "{" , "Bad Parse")
        self.next()


        offsetVec = self.parseOffset()
        self.mOffsets.append(offsetVec)
        self.next()
        channelParseREsult =self.parseChannels()
        self.mJointChannels.append(channelParseREsult)
        self.next()
        while self.cur() == "JOINT" or self.cur() == "End":
            if self.cur() == "JOINT":
                self.parseJoint()
            else :
                if self.cur() != "End":
                    self.errorString( False, "Bad Parse")
                if self.next() != "Site":
                    self.errorString( False, "Bad Parse")
                self.errorString( self.next() == '{', "Bad Parse")
                self.next()
                offsetVec = self.parseOffset()
                self.errorString( self.next() == '}', "Bad Parse")
                self.mJointNames.append(self.mJointNames[-1]+"_end")
                self.mJointParents.append(self.mJointStack[-1])
                self.mOffsets.append(offsetVec)
                self.mJointChannels.append([])

            self.next()

        self.errorString( self.cur() == '}',"Bad Parse")
        self.mJointStack.pop()

    def parseMotion(self):
        self.errorString(self.cur() == "MOTION","Bad Parse Motion Section")
        self.errorString(self.next() == "Frames:" and self.is_number(self.next()),"Bad Parse Frames")
        self.mFrameNum = int(self.cur())
        self.errorString(self.next() == "Frame" and self.next() == "Time:" and self.is_number(self.next()),"Bad Parse Frames")
        self.mFrameTime = float(self.cur())
        for frameIdx in range(0,self.mFrameNum):
            self.mJointFrameMotionData.append([])
            for joint in range(0,len(self.mJointChannels)):
                channelData = []
                for channel in range(0,len(self.mJointChannels[joint])):
                    self.errorString(self.is_number(self.next()),"Bad Parse Motion Data in Joint {self.mJointNames[joint]}" )
                    channelData.append(float(self.cur()))
                self.mJointFrameMotionData[frameIdx].append(channelData)

    def parse(self):
        ##DFA here
        self.next()
        self.parseHierarchy()
        self.next()
        self.parseMotion()

def main():
    reader = BvhReader(r"data/walk60.bvh")
    reader.parse()

if __name__ == "__main__":
    main()

