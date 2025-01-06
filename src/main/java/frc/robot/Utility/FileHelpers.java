package frc.robot.Utility;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

public class FileHelpers {

    /**
     * Reads each line of a file as plain text
     * 
     * @param path The file path location to find the file at. For the robot this is
     *             usually /home/lvuser/...
     * @return A string containing each character in the file
     */
    public static String readFile(String path) {
        String contents = "";
        try {
            File file = new File(path);
            Scanner reader = new Scanner(file);
            while (reader.hasNextLine()) {
                String data = reader.nextLine();
                contents += "\n" + data;
            }
            reader.close();
        } catch (FileNotFoundException e) {
            System.out.println("File not found at: " + path);
            e.printStackTrace();
        }
        return contents;
    }

    /**
     * Writes a String of content to a file at the given path
     * 
     * @param path    The file path location to find the file at. For the robot this
     *                is usually /home/lvuser/...
     * @param content A string containing the content to write out to the file
     */
    public static void writeFile(String path, String content) {
        try {
            FileWriter writer = new FileWriter(path);
            writer.write(content);
            writer.close();
        } catch (IOException e) {
            System.out.println("An error occurred while writing to a file at: " + path);
            e.printStackTrace();
        }
    }

    /**
     * Reads in a .csv file at the given file path, splits the file at each new line
     * and returns a 2d double array where
     * the rows are the new lines and columns are the values
     * 
     * @param path The file path location to find the file at. For the robot this is
     *             usually /home/lvuse/...
     * @return A double array containing the values in the csv
     */
    public static double[][] parseCSV(String path) {
        // Read in the file and split at each new line
        String file = readFile(path);
        String[] fileSetpoints = file.split("\n");
        
        fileSetpoints[1] = "";

        // Parse plain text strings into doubles and places them in the output array
        //fileSetpoints.length - 1, fileSetpoints.length - 1
        double[][] outputArray = new double[(int)(fileSetpoints.length - 1)][(int)(fileSetpoints[2].split(",").length)];
        int i = 0;
        int j = 0;
        for (String x : fileSetpoints) {
            if (!x.equals("")) {
                j = 0;
                for (String y : x.split(",")) {
                    if (!x.equals("")){
                        outputArray[i][j] = Double.parseDouble(y);
                        j += 1;
                    }
                }
                i += 1;
            }
        }

        return outputArray;
    }
}
