import cv2
import numpy as np
import os

# グローバル変数
points = []
output_directory = "/mnt/c/Users/motti/Desktop/Carck_ros2/Output_Images"  # 保存先ディレクトリを指定
image_count = 1  # 画像番号の管理用変数
current_result_image = None  # 最も長い線を保存するためのグローバル変数

def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(points) < 4:
            points.append((x, y))
            cv2.circle(param, (x, y), 5, (0, 255, 0), -1)
            cv2.imshow("Image", param)
            if len(points) == 4:
                print("4 points selected:", points)

def extract_test_piece(image):
    global points
    points = []
    resized_image = resize_func(image.copy())  # 画像をリサイズ

    # 画像を表示し、マウスコールバックを設定
    cv2.imshow("Image", resized_image)
    cv2.setMouseCallback("Image", click_event, resized_image)
    cv2.waitKey(0)

    if len(points) == 4:
        # 透視変換を実行
        transformed_image = warp_perspective(resized_image, points)
        cv2.imshow("Warped Image", transformed_image)
        create_trackbar(transformed_image)
        return transformed_image
    else:
        raise ValueError("Four points are required for perspective conversion")

def warp_perspective(image, points):
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [480, 0], [480, 480], [0, 480]])  # 固定サイズにワープ
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    warped_img = cv2.warpPerspective(image, matrix, (480, 480))
    return warped_img

def detect_and_measure_lines(img, canny_thresh1, canny_thresh2, max_line_gap, scale=20):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, canny_thresh1, canny_thresh2, apertureSize=3)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=100, minLineLength=30, maxLineGap=max_line_gap)

    longest_line = None
    max_length = 0

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            if length > max_length:
                max_length = length
                longest_line = (x1, y1, x2, y2)

        if longest_line:
            x1, y1, x2, y2 = longest_line
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
            length_cm = (max_length / img.shape[1]) * scale
            cv2.putText(img, f"Length: {length_cm:.2f} cm", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
            print(f"Longest line length in cm: {length_cm:.2f}")

    return img

def create_trackbar(warped_img):
    cv2.namedWindow('Warped Image')
    # トラックバーを作成
    cv2.createTrackbar('Brightness', 'Warped Image', 100, 200, lambda x: None)
    cv2.createTrackbar('Contrast', 'Warped Image', 100, 300, lambda x: None)
    cv2.createTrackbar('Canny Thresh 1', 'Warped Image', 100, 500, lambda x: None)
    cv2.createTrackbar('Canny Thresh 2', 'Warped Image', 200, 500, lambda x: None)
    cv2.createTrackbar('Max Line Gap', 'Warped Image', 20, 150, lambda x: None)

    while True:
        update_image(warped_img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            save_image()  # 画像保存処理を呼び出す
            break
        elif key == ord('0'):  # 0が押された場合
            print("Process terminated by user pressing 0.")
            exit_program()  # 強制終了関数を呼び出す
            break

    cv2.destroyAllWindows()

def exit_program():
    """全てのプロセスを終了するための関数"""
    print("Exiting all processes...")
    cv2.destroyAllWindows()
    os._exit(0)  # プログラムを強制終了

def update_image(warped_img):
    brightness = cv2.getTrackbarPos('Brightness', 'Warped Image') - 100
    contrast = cv2.getTrackbarPos('Contrast', 'Warped Image') / 100.0
    canny_threshold1 = cv2.getTrackbarPos('Canny Thresh 1', 'Warped Image')
    canny_threshold2 = cv2.getTrackbarPos('Canny Thresh 2', 'Warped Image')
    max_line_gap = cv2.getTrackbarPos('Max Line Gap', 'Warped Image')

    temp_image = cv2.convertScaleAbs(warped_img, alpha=contrast, beta=brightness)
    result_image = detect_and_measure_lines(temp_image.copy(), canny_threshold1, canny_threshold2, max_line_gap, scale=20)

    cv2.imshow('Warped Image', result_image)

    # グローバル変数に現在の処理済み画像を保存
    global current_result_image
    current_result_image = result_image

def save_image():
    global current_result_image, image_count
    if current_result_image is not None:
        if not os.path.exists(output_directory):
            os.makedirs(output_directory)
        file_name = os.path.join(output_directory, f'processed_image{image_count}.png')  # 連番付きファイル名
        cv2.imwrite(file_name, current_result_image)
        print(f"Image saved to {file_name}")
        image_count += 1  # 画像番号をインクリメント
    else:
        print("No image to save.")

def resize_func(img):
    resized_img = cv2.resize(img, (480, 480), interpolation=cv2.INTER_AREA)
    return resized_img
