package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import java.util.ArrayList;

public class Utils {
    private static ArrayList<Pair<Double, String>> timers;

    public static void clear() {
        timers = new ArrayList<>();
    }

    public static boolean hasTimer(String id) {
        for (Pair<Double, String> timer: timers) {
            if (timer.second.equals(id))
                return true;
        }

        return false;
    }

    public static double getTime(String id) {
        for (Pair<Double, String> timer: timers) {
            if (timer.second.equals(id))
                return timer.first;
        }

        return -1;
    }

    public static boolean timer(double time, String id) {
        double currentTime = System.nanoTime() / 1E9;

        for (Pair<Double, String> timer: timers) {
            if (timer.second.equals(id))
                return time > (currentTime - timer.first);
        }

        timers.add(new Pair<>(currentTime, id));
        return true;
    }
}
