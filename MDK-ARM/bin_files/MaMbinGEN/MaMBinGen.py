# include initialization code
import hashlib
import os
import binascii
import sys, getopt


class MakeHeader:

    PACKET_SIZE = 72
    sourcePath = ""
    destPath = ""
    srcFile = ""
    destFile = ""
    startLoad = ""

    def __init__(self, startLoad, sPath, sFlName, dPath, dFlName=""):
        self.PACKET_SIZE = 72
        self.startLoad = startLoad
        self.sourcePath = sPath
        self.destPath = dPath
        self.srcFile = sFlName
        if dFlName == "":
            dFlName = "_" + self.srcFile
        self.destFile = dFlName


    def getSize(self, fileobject):
        fileobject.seek(0, 2)  # move the cursor to the end of the file
        size = fileobject.tell()
        return size


    def AutoPadding(self, name_file, name_file_out):
        file = open(name_file, 'rb')
        size_ = self.getSize(file)
        file.close()

        if not ((size_ % self.PACKET_SIZE) == 0):
            file = open(name_file, 'rb')
            bytes = file.read(size_)
            file.close()

            newFile = open(name_file_out, "wb")
            # write to file
            newFile.write(bytes)
            for item in range(1, self.PACKET_SIZE - (size_ % self.PACKET_SIZE) + 1):  # write padding
                newFile.write(chr(255))
            newFile.close()
        else:
            file = open(name_file, 'rb')
            bytes = file.read(size_)
            file.close()

            newFile = open(name_file_out, "wb")
            # write to file
            newFile.write(bytes)
            newFile.close()


    def getMD5(self, nome_file, size):
        init_string = "0000"
        dgdt_string = "07EE0C1F173B3B00"        # dt = 31-12-2030 23:59:59

        md5 = hashlib.md5()
        with open(nome_file, mode='rb') as f:
            d = hashlib.md5()
            #d.update(binascii.unhexlify(init_string))
            buf = f.read(size)
            d.update(buf)
            #d.update(binascii.unhexlify(dgdt_string))
            return d.hexdigest()
        return ""


    def CreateHeader(self):
        tmpFile = self.destPath + "/" + self.destFile + ".padding_"
        self.AutoPadding(self.sourcePath + "/" + self.srcFile, tmpFile)

        file = open(tmpFile, 'rb')
        size_ = self.getSize(file)
        file.close()

        file = open(tmpFile, 'rb')
        bytes = file.read(size_)
        file.close()

        size_file_padding_ = "%08X" % size_

        os.remove(tmpFile)

        tmpFile = self.destPath + "/" + self.destFile + ".md5_"
        newFile = open(tmpFile, "wb")
        # write to file
        newFile.write(chr(0))
        newFile.write(chr(0))
        newFile.write(bytes)
        newFile.write(chr(7))
        newFile.write(chr(238))#228))
        newFile.write(chr(12))
        newFile.write(chr(31))
        newFile.write(chr(23))
        newFile.write(chr(59))
        newFile.write(chr(59))
        newFile.write(chr(0))
        newFile.close()

        file = open(tmpFile, 'rb')
        size_ = self.getSize(file)
        file.close()

        hash = self.getMD5(tmpFile, size_)

        os.remove(tmpFile)

        digest = hash[16:]
        # print "digest %s" % digest

        newFileHeader = [77, 69, 84, 69, 82, 83, 32, 65, 78, 68,
                         32, 77, 79, 82, 69, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 1, 1, 0,
                         0, 0, 0, 0, 0, int(size_file_padding_[0:2], 16), int(size_file_padding_[2:4], 16),
                         int(size_file_padding_[4:6], 16), int(size_file_padding_[6:8], 16), 0,
                         0, 0, 0, 100, 111, 119, 110, 108, 111, 97,
                         100, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                         #7, 228, 12, 31, 23, 59, 59, 0, int(digest[0:2], 16), int(digest[2:4], 16),
                         7, 238, 12, 31, 23, 59, 59, 0, int(digest[0:2], 16), int(digest[2:4], 16),
                         int(digest[4:6], 16), int(digest[6:8], 16), int(digest[8:10], 16), int(digest[10:12], 16),
                         int(digest[12:14], 16), int(digest[14:16], 16), int(self.startLoad[0:2], 16), int(self.startLoad[2:4], 16),
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        tmpFile2 = self.destPath + "/" + self.destFile
        print ' creating ' + tmpFile2 + '...'
        newFile2 = open(tmpFile2, "wb")
        # write to file
        b_array_header = bytearray(newFileHeader)
        newFile2.write(b_array_header)
        newFile2.write(bytes)
        newFile2.close()

        pass



# sourcePath = os.path.curdir + "/binaries" # "/eletra"
# destPath = os.path.curdir + "/binaries" # "/eletra"
# sourceFile = "DemoBoard_debug.bin" #"DemoBoard.bin"
# destFile = "DemoBoard_debug_app.bin" #"DemoApp.bin"
# startLoad = "0101"      # startload for board fw upgrade
# startLoadExt = "FF01"   # startload for external device fw upgrade
#
# # to create binary for Meters And More board fw
# my_header = MakeHeader(startLoad, sourcePath, sourceFile, destPath, destFile)
#
# ## to create binary for external fw
# #my_header = MakeHeader(startLoadExt, sourcePath, sourceFile, destPath, "Ext"+destFile)
#
# my_header.CreateHeader()
