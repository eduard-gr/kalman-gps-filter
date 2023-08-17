package com.gps;

public class KalmanFilter {

    private double MIN_ACCURACY = Math.pow(5f,2);
    private long timestamp = 0; // millis
    private double latitude = 0; // degree
    private double longitude = 0; // degree
    private double variance = MIN_ACCURACY; // P matrix. Initial estimate of error

    /**
     * Kalman filter processing for latitude and longitude
     *
     * newLatitude - new measurement of latitude
     * newLongitude - new measurement of longitude
     * accuracy - measurement of 1 standard deviation error in meters
     * newTimeStamp - time of measurement in millis
     */
    public double[] process(
        long timestamp,
        double speed,
        double latitude,
        double longitude
    ) {

        if(this.timestamp == 0){
            this.timestamp = timestamp; // millis
            this.latitude = latitude; // degree
            this.longitude = longitude; // degree

            return new double[] {
                this.latitude,
                this.longitude
            };
        }


        long duration = timestamp - this.timestamp;
        if (duration > 0) {
            // time has moved on, so the uncertainty in the current position increases
            this.variance += duration * speed * speed / 1000;
            this.timestamp = timestamp;
        }

        // Kalman gain matrix 'k' = Covariance * Inverse(Covariance + MeasurementVariance)
        // because 'k' is dimensionless,
        // it doesn't matter that variance has different units to latitude and longitude
        double k = this.variance / (this.variance + MIN_ACCURACY);

        this.latitude += k * (latitude - this.latitude);
        this.longitude += k * (longitude - this.longitude);

        // new Covariance matrix is (IdentityMatrix - k) * Covariance
        this.variance = (1 - k) * this.variance;

        return new double[] {
            this.latitude,
            this.longitude
        };
    }

}
