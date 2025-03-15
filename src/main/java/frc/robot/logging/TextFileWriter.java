package frc.robot.logging;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;

public class TextFileWriter {

    public static void writeToFile(String fileName, String data) {
        File file = new File(Filesystem.getDeployDirectory(), fileName);
        
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(file))) {
            writer.write(data);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public static void main(String[] args) {
        String fileName = "File writers ";
        String dataToWrite = "File Writes Test";
        writeToFile(fileName, dataToWrite);
    }
}