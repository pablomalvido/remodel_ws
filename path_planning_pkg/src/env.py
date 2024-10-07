class Env:
    def __init__(self):
        self.x_range = (0, 50)
        self.y_range = (0, 30)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle() #ox,oy,w,h,angle #Just visual info
        self.obs_rectangle_collision = [] #ox,oy,w,h #Unused for now
        self.obs_rectangle_collision_all = []
        self.obs_circle_collision = [] #ox,oy,R
        self.obs_circle_collision_all = []

    @staticmethod
    def obs_boundary():
        obs_boundary = [
            #[0, 0, 1, 30],
            #[0, 30, 50, 1],
            #[1, 0, 50, 1],
            #[50, 1, 1, 30]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            #[14, 12, 8, 2], #[x,y,w,h]
            #[18, 22, 8, 3],
            #[26, 7, 2, 12],
            #[32, 14, 10, 2]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            #[7, 12, 3],
            #[46, 20, 2],
            #[15, 5, 2],
            #[37, 7, 3],
            #[37, 23, 3]
        ]

        return obs_cir
    
    def add_rectangle(self, rect):
        self.obs_rectangle.append(rect)

    def add_circle(self, circ):
        self.obs_circle.append(circ)

    def add_boundary(self, bound):
        self.obs_boundary.append(bound)

    def add_rectangle_collision(self, rect):
        self.obs_rectangle_collision.append(rect)

    def remove_rectangle(self, index):
        self. obs_rectangle.pop(index)