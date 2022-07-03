import java.util.Arrays;
import java.io.*;

class DTW extends dataList {

    // TODO: add global constrains + early abandoning

    // TODO: maybe optimization -> FastDTW + data umbstarction => map on lower
    // quality and then back

    static String[] fileLocation = new String[] {
            "D:\\Dokumente\\PlatformIO\\Projects\\mpu6050-plotter\\resources\\data0sample DTW matrix_x.txt",
            "D:\\Dokumente\\PlatformIO\\Projects\\mpu6050-plotter\\resources\\data0sample DTW matrix_y.txt",
            "D:\\Dokumente\\PlatformIO\\Projects\\mpu6050-plotter\\resources\\data0sample DTW matrix_z.txt",

            "D:\\Dokumente\\PlatformIO\\Projects\\mpu6050-plotter\\resources\\data1 DTW matrix_x.txt",
            "D:\\Dokumente\\PlatformIO\\Projects\\mpu6050-plotter\\resources\\data1 DTW matrix_y.txt",
            "D:\\Dokumente\\PlatformIO\\Projects\\mpu6050-plotter\\resources\\data1 DTW matrix_z.txt",

            "D:\\Dokumente\\PlatformIO\\Projects\\mpu6050-plotter\\resources\\data2 DTW matrix_x.txt",
            "D:\\Dokumente\\PlatformIO\\Projects\\mpu6050-plotter\\resources\\data2 DTW matrix_y.txt",
            "D:\\Dokumente\\PlatformIO\\Projects\\mpu6050-plotter\\resources\\data2 DTW matrix_z.txt",

            "D:\\Dokumente\\PlatformIO\\Projects\\mpu6050-plotter\\resources\\data3wrong DTW matrix_x.txt",
            "D:\\Dokumente\\PlatformIO\\Projects\\mpu6050-plotter\\resources\\data3wrong DTW matrix_y.txt",
            "D:\\Dokumente\\PlatformIO\\Projects\\mpu6050-plotter\\resources\\data3wrong DTW matrix_z.txt",
    };

    public static void main(String[] args) {
        clearResourceData();
        int dataCount = 0;
        int fileLocationCounter = 0;

        for (double[][] patternData : dataPacks) {
            System.out.println("Data " + dataCount++ + ": ");
            int xyz = 0;
            int tmpSimilarity = 0;

            for (double[] dataSeq : patternData) {
                intitialize();
                calcualteCostMatrix(dataPacks[0][xyz % 3], dataSeq);
                // printFile(fileLocation[fileLocationCounter++]);

                // printSimilarityOutput(dataPacks[0][xyz % 3], dataSeq);

                if (xyz % 3 != 0) {// igonre x axis

                    double tmpSpace = calculateSpaceToDia(dataPacks[0][xyz % 3].length - 1,
                            dataSeq.length - 1);
                    tmpSimilarity += tmpSpace;

                    // System.out.println(
                    // "PathCost: "
                    // + calculatePathCostSum(dataPacks[0][xyz % 3].length - 1, dataSeq.length -
                    // 1));

                    // System.out.println("Space : " + tmpSpace);
                }

                xyz++; // iterate over axies
            }
            if (tmpSimilarity < 10000) {
                System.out.println("Successfully recognized");
            } else {
                System.out.println("Wrong gesture");
            }
            // System.out.println("Similarity: " + tmpSimilarity + "\n");
        }
    }

    static void clearResourceData() {
        for (int i = 0; i < fileLocation.length; i++) {
            new File(fileLocation[i]).delete();
        }
    }

    static void calcualteCostMatrix(double[] tempPattern, double[] seqPattern) {
        for (int i = 1; i < tempPattern.length; i++)
            for (int j = 1; j < seqPattern.length; j++)
                costMatrix[i][j] = calculateCost(i, j, tempPattern, seqPattern);
    }

    private static void printFile(String fileLocation) {
        try {
            BufferedWriter writer = new BufferedWriter(
                    new FileWriter(
                            fileLocation));
            writer.write(printMatrix());
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        System.out.println("Successfully wirtten");
    }

    static void printSimilarityOutput(double[] tempPattern, double[] seqPattern) {
        double spaceDiagonale = calculateSpaceToDia(tempPattern.length - 1, seqPattern.length - 1);

        System.out.println("spacing diagonal: " + spaceDiagonale);
    }

    static void intitialize() {
        Arrays.stream(costMatrix)
                .forEach(row -> Arrays.stream(row)
                        .forEach(column -> column = Double.POSITIVE_INFINITY));
        costMatrix[0][0] = 0;
    }

    static double calculateSpaceToDia(int i, int j) { // calculate shortest distance between point/diagonale
        double tmpResult = 0;

        while (i >= 0 && j >= 0) {
            double tmpChilds[] = minChilds(i, j);
            double tmpMin = getMin(i, j);

            double tmpCoord = 0.5 * (i + j);
            double tmpDistance = Math.sqrt(Math.pow(tmpCoord - i, 2) + Math.pow(tmpCoord - j, 2));

            // if (tmpDistance > 11)// NOTE -> global constrain
            // return -1;

            tmpResult += Math.abs(tmpDistance);

            if (tmpMin != tmpChilds[1] || tmpChilds[0] == tmpChilds[1])
                --j;
            if (tmpMin != tmpChilds[2] || tmpChilds[0] == tmpChilds[2])
                --i;
        }

        return Math.round(tmpResult * 10.0) / 10.0;
    }

    static int countPathDiagonal(int i, int j) {
        int tmpCount = 0;
        while (i >= 0 && j >= 0) {
            double tmpChilds[] = minChilds(i, j);
            double tmpMin = getMin(i, j);
            boolean downFlag = false;
            boolean leftFlag = false;

            if (tmpMin != tmpChilds[1] || tmpChilds[0] == tmpChilds[1]) {
                --j;
                leftFlag = true;
            }
            if (tmpMin != tmpChilds[2] || tmpChilds[0] == tmpChilds[2]) {
                --i;
                downFlag = true;
            }

            if (downFlag && leftFlag)
                ++tmpCount;
        }
        return tmpCount;
    }

    static double getMin(int i, int j) { // min -> match, insertion, deletion
        return Arrays.stream(minChilds(i, j)).min().getAsDouble();
    }

    private static double[] minChilds(int i, int j) {
        double tmpChilds[] = { Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY };

        if (i - 1 >= 0 && j - 1 >= 0) // match
            tmpChilds[0] = costMatrix[i - 1][j - 1];
        if (i - 1 >= 0) // insertion
            tmpChilds[1] = costMatrix[i - 1][j];
        if (j - 1 >= 0) // deletion
            tmpChilds[2] = costMatrix[i][j - 1];
        return tmpChilds;
    }

    static double calculateCost(int i, int j, double[] template, double[] matrix) { // dtw algorithm
        return Math.abs(template[i] - matrix[j]) + getMin(i, j);
    }

    // NOTE => useless
    static int calculateDistance(int i, int j) { // iterative addition of the path length
        int tmpDistance = 0;

        while (i >= 0 && j >= 0) {
            double tmpChilds[] = minChilds(i, j);
            double tmpMin = getMin(i, j);

            if (tmpMin != tmpChilds[1] || tmpChilds[0] == tmpChilds[1])
                --j;
            if (tmpMin != tmpChilds[2] || tmpChilds[0] == tmpChilds[2])
                --i;
            ++tmpDistance;
        }

        return tmpDistance;
    }

    // NOTE => useless
    static double calculatePathCostSum(int i, int j) { // iterative addition of the path length
        double tmpCost = 0;

        while (i >= 0 && j >= 0) {
            double tmpChilds[] = minChilds(i, j);
            double tmpMin = getMin(i, j);

            tmpCost += costMatrix[i][j];

            if (tmpMin != tmpChilds[1] || tmpChilds[0] == tmpChilds[1])
                --j;
            if (tmpMin != tmpChilds[2] || tmpChilds[0] == tmpChilds[2])
                --i;
        }

        return tmpCost;
    }

    public static String printMatrix() {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < costMatrix.length; i++) {
            for (int j = 0; j < costMatrix[i].length; j++) {
                builder.append(Math.round(costMatrix[i][j] * 10.0) / 10.0 + "\t");
            }
            builder.append("\n");
        }
        return builder.toString();
    }
}