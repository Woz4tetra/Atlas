
class BayesFilter():
    def __init__(self, map):
        self.map = map  # GPS map of a course

        self.num_samples = 1000
        self.redistrib_factor = 0.3
        self.particle_cut_off = 0.5

        # Uniform sampling of num_samples points from the course
        # a list of indices from the map
        self.particles =

        # the weight of each particle (smaller numbers are better here)
        self.particle_weights =

    def take_measurement(self, gps_coord):
        meas_lat, meas_lon = gps_coord
        for index in range(len(self.particles)):
            lat, lon = self.map[self.particles[index]]

            # weights determined by distance from measurement
            # Smaller distances are better (weighted higher)
            self.particle_weights[index] = ((lat - meas_lat) ** 2 + (lon - meas_lon) ** 2) ** 0.5

        likely_weight = min(self.particle_weights)
        locations = []
        for index, weight in enumerate(self.particle_weights):
            if weight == likely_weight:
                locations.append(self.map[self.particles[index]])

        return locations

    def reweight_particles(self):
        unlikely_weight = max(self.particle_weights)
        self.particles = []

        for index, weight in enumerate(self.particle_weights):
            if self.particle_cut_off < weight / unlikely_weight:
                self.particles.append(index)



    def transition(self, sensor_stuff):
        # resample (1 - redistrib_factor)% of the points
        # redistribute redistrib_factor% of the points randomly

        # using sensor data, determined how many indices the system moved
        moved_by = ??

