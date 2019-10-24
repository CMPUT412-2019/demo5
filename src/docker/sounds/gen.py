from gtts import gTTS
from tqdm import tqdm

if __name__ == '__main__':
    file_contents = {
        '{}.mp3'.format(i):  'Found marker {}'.format(i) for i in range(51)
    }

    for filename, text in tqdm(file_contents.iteritems(), total=len(file_contents)):
        gTTS(text=text).save(filename)