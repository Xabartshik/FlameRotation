using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using System.Windows;
using System.Windows.Media.Media3D;

namespace FlameRotation
{
    public partial class MainWindow : Window
    {
        public static double scale = 0.66;
        public static int triangle_count = 16;

        private Point3D[] cubeVertices;
        private int[][] cubeFaces;
        private Brush[] faceBrushes;

        private List<Triangle> fireTriangles = new List<Triangle>();
        private List<Particle> particles = new List<Particle>();
        private Point3D[] transformedCubeVertices;
        private Random random = new Random();
        private DispatcherTimer timer;
        private int tick_counter = 0;
        private double angleX = 0;
        private double angleY = 0;
        private double angleZ = 0;
        private List<SmallTriangle> smallTriangles = new List<SmallTriangle>();

        




        public MainWindow()
        {
            InitializeComponent();

            // Инициализация куба
            InitializeCube();

            // Инициализация огня
            InitializeFire();

            // Настройка таймера для анимации
            timer = new DispatcherTimer();
            timer.Interval = TimeSpan.FromMilliseconds(30);
            timer.Tick += Timer_Tick;
            timer.Start();
        }

        private void InitializeCube()
        {
            // Вершины куба
            cubeVertices = new Point3D[]
            {
                new Point3D(-1, -1, -1),//0
                new Point3D(1, -1, -1),//1
                new Point3D(1, 1, -1),//2
                new Point3D(-1, 1, -1),//3
                new Point3D(-1, -1, 1),//4
                new Point3D(1, -1, 1),//5
                new Point3D(1, 1, 1),//6
                new Point3D(-1, 1, 1)//7
            };

            transformedCubeVertices = new Point3D[]
{
                new Point3D(-1, -1, -1),//0
                new Point3D(1, -1, -1),//1
                new Point3D(1, 1, -1),//2
                new Point3D(-1, 1, -1),//3
                new Point3D(-1, -1, 1),//4
                new Point3D(1, -1, 1),//5
                new Point3D(1, 1, 1),//6
                new Point3D(-1, 1, 1)//7
};

            // Грани куба
            cubeFaces = new int[][]
            {
                new int[] {0, 1, 2, 3}, // задняя грань
                new int[] {4, 5, 6, 7}, // передняя грань
                new int[] {0, 1, 5, 4}, // нижняя грань
                new int[] {2, 3, 7, 6}, // верхняя грань
                new int[] {0, 3, 7, 4}, // левая грань
                new int[] {1, 2, 6, 5}  // правая грань
            };

            // Цвета граней
            faceBrushes = new Brush[]
            {
                Brushes.Transparent, // задняя грань
                Brushes.Transparent, // передняя грань
                Brushes.Black,       // нижняя грань
                Brushes.Transparent, // верхняя грань
                Brushes.Transparent, // левая грань
                Brushes.Transparent  // правая грань
            };
        }

        private void InitializeFire()
        {
            // Центр нижней грани
            double centerX = 0.0; // Среднее X-координат грани
            double centerY = -1;  // Y-координата нижней грани
            double centerZ = 0.0; // Среднее Z-координат грани

            // Размер треугольников (масштаб)
            double scale = 0.66; // Уменьшаем размер треугольников

            // Создаем треугольники для огня
            for (int i = 0; i < triangle_count; i++)
            {
                // Случайный угол поворота вокруг оси Y
                double angle = random.NextDouble() * Math.PI * 2;

                // Вершины треугольника
                var vertex1 = new Point3D(
                    centerX + Math.Cos(angle) * scale, // X
                    centerY + 0.01,                           // Y (нижняя грань)
                    centerZ + Math.Sin(angle) * scale  // Z
                );

                var vertex2 = new Point3D(
                    centerX + Math.Cos(angle + Math.PI / 3) * scale, // X
                    centerY + 0.01,                                       // Y (нижняя грань)
                    centerZ + Math.Sin(angle + Math.PI / 3) * scale  // Z
                );

                var vertex3 = new Point3D(
                    centerX,                // X (центр)
                    centerY + 0.5 * scale,  // Y (верхняя вершина)
                    centerZ                 // Z (центр)
                );

                fireTriangles.Add(new Triangle(vertex1, vertex2, vertex3));

                fireTriangles.Add(new Triangle(vertex1, vertex2, vertex3));

                // Создаем маленькие красные треугольники
                var smallVertex1 = new Point3D(
                    centerX + Math.Cos(angle) * scale * 0.5,
                    centerY + 0.01,
                    centerZ + Math.Sin(angle) * scale * 0.5
                );

                var smallVertex2 = new Point3D(
                    centerX + Math.Cos(angle + Math.PI / 3) * scale * 0.5,
                    centerY + 0.01,
                    centerZ + Math.Sin(angle + Math.PI / 3) * scale * 0.5
                );

                var smallVertex3 = new Point3D(
                    centerX,
                    centerY + 0.5 * scale * 0.5,
                    centerZ
                );

                smallTriangles.Add(new SmallTriangle(smallVertex1, smallVertex2, smallVertex3));
            }
        }

        private void Timer_Tick(object sender, EventArgs e)
        {
            // Обновление углов поворота
            if (checkBoxX.IsChecked == true) angleX += 0.05;
            if (checkBoxY.IsChecked == true) angleY += 0.05;
            if (checkBoxZ.IsChecked == true) angleZ += 0.05;
            tick_counter++;
            // Очистка холста
            canvas.Children.Clear();

            // Обновление и отрисовка частиц
            UpdateParticles();
            DrawParticles();

            // Отрисовка куба (включая черную грань)
            DrawCube();

            // Проверка, находится ли пламя выше черной грани
            if (!IsBlackFaceVisible())
            {   
                if (tick_counter % 2 == 0)
                // Обновление и отрисовка огня
                UpdateFire();
                DrawFire();
            }

        }

        private bool IsBlackFaceVisible()
        {
            // Вершины черной грани (нижней грани куба)
            var v0 = transformedCubeVertices[0]; // (-1, -1, -1)
            var v1 = transformedCubeVertices[1]; // (1, -1, -1)
            var v2 = transformedCubeVertices[5]; // (1, -1, 1)

            // Векторы двух ребер грани
            var edge1 = new Vector3D(v1.X - v0.X, v1.Y - v0.Y, v1.Z - v0.Z);
            var edge2 = new Vector3D(v2.X - v0.X, v2.Y - v0.Y, v2.Z - v0.Z);

            // Векторное произведение edge1 и edge2 (нормаль грани)
            var normal = new Vector3D(
                edge1.Y * edge2.Z - edge1.Z * edge2.Y, // X
                edge1.Z * edge2.X - edge1.X * edge2.Z, // Y
                edge1.X * edge2.Y - edge1.Y * edge2.X  // Z
            );
            normal.Normalize(); // Нормализуем вектор

            // Вектор направления камеры (от грани к камере)
            var cameraPosition = new Vector3D(0, 0, 10); // Позиция камеры
            var cameraDirection = new Vector3D(
                cameraPosition.X - v0.X,
                cameraPosition.Y - v0.Y,
                cameraPosition.Z - v0.Z
            );
            cameraDirection.Normalize();

            // Скалярное произведение нормали и направления камеры
            double dotProduct = normal.X * cameraDirection.X +
                                normal.Y * cameraDirection.Y +
                                normal.Z * cameraDirection.Z;

            // Если скалярное произведение > 0, грань видима
            return dotProduct > 0;
        }

        private void DrawCube()
        {
            var rotationX = GetRotationMatrixX(angleX);
            var rotationY = GetRotationMatrixY(angleY);
            var rotationZ = GetRotationMatrixZ(angleZ);

            for (int i = 0; i < cubeVertices.Length; i++)
            {
                var point = cubeVertices[i];
                point = MultiplyMatrixVector(rotationX, point);
                point = MultiplyMatrixVector(rotationY, point);
                point = MultiplyMatrixVector(rotationZ, point);
                transformedCubeVertices[i] = point;
            }
            // Отрисовка ребер куба
            DrawEdges(transformedCubeVertices);
            // Отрисовка граней куба
            for (int i = 0; i < cubeFaces.Length; i++)
            {
                var face = cubeFaces[i];
                var points = face.Select(index => Project(transformedCubeVertices[index])).ToArray();
                DrawFace(points, faceBrushes[i]);
            }

        }

        private void DrawEdges(Point3D[] vertices)
        {
            // Ребра куба (индексы вершин)
            var edges = new int[][]
            {
                new int[] {0, 1}, // задняя грань
                new int[] {1, 2},
                new int[] {2, 3},
                new int[] {3, 0},
                new int[] {4, 5}, // передняя грань
                new int[] {5, 6},
                new int[] {6, 7},
                new int[] {7, 4},
                new int[] {0, 4}, // соединение задней и передней граней
                new int[] {1, 5},
                new int[] {2, 6},
                new int[] {3, 7}
            };

            // Отрисовка каждого ребра
            foreach (var edge in edges)
            {
                var startPoint = Project(vertices[edge[0]]);
                var endPoint = Project(vertices[edge[1]]);

                var line = new Line
                {
                    X1 = startPoint.X,
                    Y1 = startPoint.Y,
                    X2 = endPoint.X,
                    Y2 = endPoint.Y,
                    Stroke = Brushes.Black, // Цвет ребер
                    StrokeThickness = 1 // Толщина ребер
                };
                canvas.Children.Add(line);
            }
        }

        private void UpdateFire()
        {
            // Обновление положения вершин треугольников огня
            foreach (var triangle in fireTriangles)
            {
                triangle.Update(random);
            }
            foreach (var triangle in smallTriangles)
            {
                triangle.Update(random);
            }
        }

        private void DrawFire()
        {
            var rotationX = GetRotationMatrixX(angleX);
            var rotationY = GetRotationMatrixY(angleY);
            var rotationZ = GetRotationMatrixZ(angleZ);

            // Отрисовка треугольников огня
            foreach (var triangle in fireTriangles)
            {
                // Применяем поворот к вершинам треугольника
                var vertex1 = MultiplyMatrixVector(rotationX, triangle.Vertex1);
                vertex1 = MultiplyMatrixVector(rotationY, vertex1);
                vertex1 = MultiplyMatrixVector(rotationZ, vertex1);

                var vertex2 = MultiplyMatrixVector(rotationX, triangle.Vertex2);
                vertex2 = MultiplyMatrixVector(rotationY, vertex2);
                vertex2 = MultiplyMatrixVector(rotationZ, vertex2);

                var vertex3 = MultiplyMatrixVector(rotationX, triangle.Vertex3);
                vertex3 = MultiplyMatrixVector(rotationY, vertex3);
                vertex3 = MultiplyMatrixVector(rotationZ, vertex3);

                // Проецируем вершины на 2D
                var points = new Point[]
                {
            Project(vertex1),
            Project(vertex2),
            Project(vertex3)
                };

                // Отрисовка треугольника
                DrawFace(points, Brushes.Orange);
            }

            // Отрисовка маленьких красных треугольников
            foreach (var smallTriangle in smallTriangles)
            {
                var smallVertex1 = MultiplyMatrixVector(rotationX, smallTriangle.Vertex1);
                smallVertex1 = MultiplyMatrixVector(rotationY, smallVertex1);
                smallVertex1 = MultiplyMatrixVector(rotationZ, smallVertex1);

                var smallVertex2 = MultiplyMatrixVector(rotationX, smallTriangle.Vertex2);
                smallVertex2 = MultiplyMatrixVector(rotationY, smallVertex2);
                smallVertex2 = MultiplyMatrixVector(rotationZ, smallVertex2);

                var smallVertex3 = MultiplyMatrixVector(rotationX, smallTriangle.Vertex3);
                smallVertex3 = MultiplyMatrixVector(rotationY, smallVertex3);
                smallVertex3 = MultiplyMatrixVector(rotationZ, smallVertex3);

                var smallPoints = new Point[]
                {
            Project(smallVertex1),
            Project(smallVertex2),
            Project(smallVertex3)
                };

                DrawFace(smallPoints, Brushes.Yellow);
            }
        }

        private void UpdateParticles()
        {
            // Добавление новых частиц
            if (random.NextDouble() < 0.5)
            {
                var radius = random.NextDouble() * 0.5;
                var angle = random.NextDouble() * Math.PI * 2;
                var x = radius * Math.Cos(angle);
                var z = radius * Math.Sin(angle);
                var y = random.NextDouble() * 2 - 1;

                particles.Add(new Particle(
                    new Point3D(x, y, z),
                    new Vector3D(0, random.NextDouble() * 0.1, 0)
                ));
            }

            // Обновление положения частиц
            foreach (var particle in particles)
            {
                particle.Update();
            }

            // Удаление "мертвых" частиц
            particles.RemoveAll(p => p.IsDead);
        }

        private void DrawParticles()
        {
            var rotationX = GetRotationMatrixX(angleX);
            var rotationY = GetRotationMatrixY(angleY);
            var rotationZ = GetRotationMatrixZ(angleZ);

            // Отрисовка частиц
            foreach (var particle in particles)
            {
                // Применяем поворот к позиции частицы
                var position = MultiplyMatrixVector(rotationX, particle.Position);
                position = MultiplyMatrixVector(rotationY, position);
                position = MultiplyMatrixVector(rotationZ, position);

                // Проецируем позицию на 2D
                var point = Project(position);

                // Отрисовка частицы
                var ellipse = new Ellipse
                {
                    Width = 5,
                    Height = 5,
                    Fill = Brushes.Yellow,
                    Opacity = particle.Life
                };
                Canvas.SetLeft(ellipse, point.X);
                Canvas.SetTop(ellipse, point.Y);
                canvas.Children.Add(ellipse);
            }
        }

        private Point Project(Point3D point)
        {
            double scale = 500;
            double distance = 5;
            double factor = scale / (distance - point.Z);
            return new Point(
                canvas.ActualWidth / 2 + point.X * factor,
                canvas.ActualHeight / 2 - point.Y * factor
            );
        }

        private double[,] GetRotationMatrixX(double angle)
        {
            return new double[,]
            {
                {1, 0, 0},
                {0, Math.Cos(angle), -Math.Sin(angle)},
                {0, Math.Sin(angle), Math.Cos(angle)}
            };
        }

        private double[,] GetRotationMatrixY(double angle)
        {
            return new double[,]
            {
                {Math.Cos(angle), 0, Math.Sin(angle)},
                {0, 1, 0},
                {-Math.Sin(angle), 0, Math.Cos(angle)}
            };
        }

        private double[,] GetRotationMatrixZ(double angle)
        {
            return new double[,]
            {
                {Math.Cos(angle), -Math.Sin(angle), 0},
                {Math.Sin(angle), Math.Cos(angle), 0},
                {0, 0, 1}
            };
        }

        private Point3D MultiplyMatrixVector(double[,] matrix, Point3D point)
        {
            return new Point3D(
                matrix[0, 0] * point.X + matrix[0, 1] * point.Y + matrix[0, 2] * point.Z,
                matrix[1, 0] * point.X + matrix[1, 1] * point.Y + matrix[1, 2] * point.Z,
                matrix[2, 0] * point.X + matrix[2, 1] * point.Y + matrix[2, 2] * point.Z
            );
        }

        private void DrawFace(Point[] points, Brush fillBrush)
        {
            var polygon = new Polygon
            {
                Points = new PointCollection(points),
                Fill = fillBrush,
                Stroke = Brushes.Transparent, // Убираем обводку
                StrokeThickness = 0
            };
            canvas.Children.Add(polygon);
        }

        private class Triangle
        {
            public Point3D Vertex1 { get; set; }
            public Point3D Vertex2 { get; set; }
            public Point3D Vertex3 { get; set; }

            public Triangle(Point3D v1, Point3D v2, Point3D v3)
            {
                Vertex1 = v1;
                Vertex2 = v2;
                Vertex3 = v3;
            }

            public void Update(Random random)
            {
                //// Изменение положения вершин для создания эффекта пламени
                //Vertex1.Y += random.NextDouble() * 0.1 - 0.05;
                //Vertex2.Y += random.NextDouble() * 0.1 - 0.05;
                Vertex3.Y = (random.NextDouble() * 2 - 1) * scale - 0.05;
                Vertex3.X = (random.NextDouble() * 2 - 1) * 0.3;
                Vertex3.Z = (random.NextDouble() * 2 - 1) * 0.3;
            }
        }

        private class SmallTriangle
        {
            public Point3D Vertex1 { get; set; }
            public Point3D Vertex2 { get; set; }
            public Point3D Vertex3 { get; set; }

            public SmallTriangle(Point3D v1, Point3D v2, Point3D v3)
            {
                Vertex1 = v1;
                Vertex2 = v2;
                Vertex3 = v3;
            }

            public void Update(Random random)
            {
                Vertex3.Y = (random.NextDouble() * 2 - 1) * (scale/4) - 0.5;
                Vertex3.X = (random.NextDouble() * 2 - 1) * 0.3;
                Vertex3.Z = (random.NextDouble() * 2 - 1) * 0.3;
            }
        }




        private class Particle
        {
            public Point3D Position { get; set; }
            public Vector3D Velocity { get; set; }
            public double Life { get; set; } = 1.0;
            public bool IsDead => Life <= 0;

            public Particle(Point3D position, Vector3D velocity)
            {
                Position = position;
                Velocity = velocity;
            }

            public void Update()
            {
                Position.X += Velocity.X;
                Position.Y += Velocity.Y;
                Position.Z += Velocity.Z;
                Life -= 0.01; // Постепенное затухание
            }
        }

        private class Point3D
        {
            public double X { get; set; }
            public double Y { get; set; }
            public double Z { get; set; }

            public Point3D(double x, double y, double z)
            {
                X = x;
                Y = y;
                Z = z;
            }
        }

    }
}