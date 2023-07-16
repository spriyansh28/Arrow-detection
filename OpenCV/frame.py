import cv2

cap = cv2.VideoCapture(0)
n_rows = 1
n_images_per_row = 11

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    height, width, ch = frame.shape

    roi_height = height / n_rows
    roi_width = width / n_images_per_row

    images = []

    for x in range(0, n_rows):
        for y in range(0,n_images_per_row):
            tmp_image=frame[x*roi_height:(x+1)*roi_height, y*roi_width:(y+1)*roi_width]
            images.append(tmp_image)

    # Display the resulting sub-frame
    for x in range(0, n_rows):
        for y in range(0, n_images_per_row):
            cv2.imshow(str(x*n_images_per_row+y+1), images[x*n_images_per_row+y])
            cv2.moveWindow(str(x*n_images_per_row+y+1), 100+(y*roi_width), 50+(x*roi_height))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()