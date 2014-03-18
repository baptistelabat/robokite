from SimpleCV import Display, JpegStreamCamera, Camera, Color
cam = JpegStreamCamera('http://192.168.1.50:8080/videofeed')
#cam = Camera(0)
disp = Display( (1000,1000))
colorToDisplay = Color.RED
while True:

  img = cam.getImage()
  img.drawRectangle(0, 0, 999, 999, color = colorToDisplay, width=0, alpha=255)
  img.save(disp)
  red_dist = img.colorDistance(color=Color.RED)
  col = red_dist.getPixel(0,0)
  #red_dist.save(disp)
  summation = col[0]+col[1]+col[2]
  print summation
  if  summation < 400:
    colorToDisplay = Color.BLUE
  else:
    colorToDisplay = Color.RED
