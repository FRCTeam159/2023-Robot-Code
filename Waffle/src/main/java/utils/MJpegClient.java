// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package utils;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.net.URLConnection;
import java.util.Arrays;
import javax.imageio.ImageIO;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import java.awt.image.DataBufferByte;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;

//====================================================
// A Mjpeg server client utility
//====================================================
// -For some reason VideoCapture is now failing to return
//  images from an http stream "CAP_IMAGES: can't find starting number"
// -Couldn't find an approriate http client utility in CameraServer
//  or cscore wppi libraries 
// -So adapted this code from MjpegStreamViewer in Smartdashboard
// 1) connects to a Mjpeg server using a url of the form: 
//    e.g. http://localhost:9000/?action=stream
// 2) reads images from the stream (from an external thread)
// 3) creates a Mat object from the image
// 4) returns the mat to the calling thread
//====================================================
public class MJpegClient {
    private static final int[] START_BYTES = new int[] { 0xFF, 0xD8 };
    private static final int[] END_BYTES = new int[] { 0xFF, 0xD9 };

    String input_url;
    InputStream stream;
    ByteArrayOutputStream imageBuffer = new ByteArrayOutputStream();

    Mat mat=null;

    public MJpegClient(String url) {
        input_url = url;
        stream = getCameraStream();
    }

    public boolean isConnected() {
        if (stream == null)
            return false;
        return true;
    }

    private InputStream getCameraStream() {
        try {
            URL url = new URL(input_url);
            URLConnection connection = url.openConnection();
            connection.setConnectTimeout(500);
            connection.setReadTimeout(5000);
            InputStream stream = connection.getInputStream();

            System.out.println("Connected to: " + input_url);
            return stream;
        } catch (Exception e) {
            System.out.println("failed to connect to: " + input_url);
        }
        return null;
    }

    public Mat read() {
        try {        
            stream.skip(stream.available());
            imageBuffer.reset();
            readUntil(stream, START_BYTES);
            Arrays.stream(START_BYTES).forEachOrdered(imageBuffer::write);
            readUntil(stream, END_BYTES, imageBuffer);
            ByteArrayInputStream  tmpStream = new ByteArrayInputStream(imageBuffer.toByteArray());
            BufferedImage im = ImageIO.read(tmpStream);
            byte[] bytes = ((DataBufferByte) im.getRaster().getDataBuffer()).getData();
            if(mat==null)
                mat = new Mat(im.getHeight(), im.getWidth(), CvType.CV_8UC3);
            mat.put(0, 0, bytes);
            return mat;
        } catch (Exception ex) {
            //System.out.println("Exception:"+ex.getMessage());
        } 
        return null;
    }

    private void readUntil(InputStream stream, int[] bytes) throws IOException {
        readUntil(stream, bytes, null);
    }

    private void readUntil(InputStream stream, int[] bytes, ByteArrayOutputStream buffer)
            throws IOException {
        for (int i = 0; i < bytes.length;) {
            int b = stream.read();
            if (b == -1) {
                throw new IOException("End of Stream reached");
            }
            if (buffer != null) {
                buffer.write(b);
            }
            if (b == bytes[i]) {
                i++;
            } else {
                i = 0;
            }
        }
    }

}
