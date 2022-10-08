# encoding=utf-8

from aip import AipOcr

import glob
from os import path
import os
from PIL import Image
import sys

# 禁用安全请求警告,当目标使用htpps时使用
 


#reload(sys)
#sys.setdefaultencoding('utf8')  # 解决UnicodeEncodeError: 'ascii' 不支持utf8的问题



def __bytes__(self):
    #return str(self).encode('iso-8859-1')
    return str(self).encode('utf-8')

APP_ID = '23561812'
API_KEY = 'IMlIn4r2epRsp4Id6SU3KXRs'
SECRET_KEY = 'GNI1uKZFm06K1DGVbFMVR4txbyT4Qnpx'

# read picture
def get_file_content(filePath):
    with open(filePath, 'rb') as fp:
        return fp.read()

# init ApiOcr
aipOcr = AipOcr(APP_ID, API_KEY, SECRET_KEY)

# define varibles
options = {
'detect_direction': 'true',
'language_type': 'CHN_ENG',
}

outfile = './export.txt'
outdir = './tmp'
if path.exists(outfile):
    os.remove(outfile)
if not path.exists(outdir):
    os.mkdir(outdir)

# call ocr
result = aipOcr.basicAccurate(get_file_content("1.jpg"))
#aipOcr.basicGeneral(get_file_content("4.jpg"))
print("识别成功！")
with open(outfile, 'a+') as fo:
    # 输出文本内容
    for text in result.get('words_result'):
        fo.writelines(str(text.get('words')) + '\n')
print("文本导出成功！")
