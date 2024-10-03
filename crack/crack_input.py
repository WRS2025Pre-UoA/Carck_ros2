import cv2
import numpy as np

def input_txt():
    # OpenCV設定
    upper_flg = False 
    img = np.zeros((720, 1280, 3))
    text=""   # パブリッシュするテキスト
    while True:
        cv2.putText(img, text, (30, 30), cv2.FONT_HERSHEY_DUPLEX,
                    1.0, (255, 255, 255))
        cv2.imshow("Text Publisher", img)

        key = cv2.waitKey(10)
        if key == 13:  # Enterキーが押された場合
            print("pressed Enter Key")
            # publish_text(text)
            cv2.destroyAllWindows()  # ウィンドウを閉じる
            break
        if key == 225:
            upper_flg = not upper_flg  # 大文字フラグを反転
            continue
        if key != -1:
            if ord('a') <= key <= ord('z') and upper_flg:
                text += chr(key).upper()
            else:
                text += chr(key)

    return text

def main():
    txt = input_txt()

    print(txt)

if __name__ == "__main__":
    main()
