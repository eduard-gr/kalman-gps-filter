import com.gps.KalmanFilter;
import org.json.JSONArray;
import org.json.JSONObject;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.IOException;
import java.net.URISyntaxException;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class KalmanFilterTest {

    @Test
    void filter() throws IOException, URISyntaxException {
        final URL resource = getClass().getClassLoader().getResource("input.json");

        Path file = Path.of(resource.toURI());
        String content = Files.readString(file);

        final var json = new JSONArray(content);
        final KalmanFilter filter = new KalmanFilter();

        final var coordinates = new JSONArray();

        for (int i=0; i < json.length(); i++) {
            final var telemetry = json.getJSONObject(i);

            double[] corrected = filter.process(
                telemetry.getLong("time"),
                telemetry.getDouble("speed") * 0.277777777778,
                telemetry.getDouble("latitude"),
                telemetry.getDouble("longitude"));

                final var tuple = new JSONArray();
                tuple.put(corrected[1]);
                tuple.put(corrected[0]);

                coordinates.put(tuple);
        }

        final var result = new JSONObject();

        result.put("coordinates", coordinates);
        result.put("type", "MultiLineString");
        System.out.println(coordinates);
    }
}