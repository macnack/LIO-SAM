#include "open3d/Open3D.h"
#include <algorithm>
#define Scans 291
#define NormalEstimationRadius 1.0
#define FeatureVoxelSize 0.5
#define FeatureRadius 2.5
#define FeatureKnn 100
#define NormalKnn 10
#define RansacNumerIteration 25000
#define RansacModelSize 3
#define RansacProbability 0.7
#define RansacMaxCorrespondenceDistanse 0.75
#define CorrespondenceCheckerDistance 0.75
#define CorrespondenceCheckerEdgeLenght 0.5
#define RansacMinCorrespondenceSetSize 25
#define MaxIcpCorrespondenceDistance 1.5
#define MinRefinementFitness 0.7
#define IcpNumIterParam 100
string PathScan = "/Downloads/REGISTRATION";
bool prepare_dataset = false;
std::shared_ptr<open3d::geometry::PointCloud> target_GlobalMap;
vector<std::shared_ptr<open3d::geometry::PointCloud>> target_scans_down;
vector<std::shared_ptr<open3d::geometry::PointCloud>> target_scans_fpfh;
Eigen::Vector3d color_target = Eigen::Vector3d(1, 0.706, 0);
Eigen::Vector3d color_source = Eigen::Vector3d(0, 0.651, 1);
Eigen::Vector3d color_source_last = Eigen::Vector3d(1, 0, 0);

class MultipleWindowsApp
{
public:
    MultipleWindowsApp()
    {
        open3d::visualization::gui::Application::GetInstance().Initialize(path_to_resources.c_str());
    }

    void Run(std::string pcd)
    {
        main_vis_1 = std::make_shared<
            open3d::visualization::visualizer::O3DVisualizer>(
            "Open3D - Mapa", WIDTH, HEIGHT);
        main_vis_1->AddAction(
            "set1",
            [this](open3d::visualization::visualizer::O3DVisualizer &)
            {
                this->get_sets1();
            });
        main_vis_1->AddAction(
            "manual",
            [this](open3d::visualization::visualizer::O3DVisualizer &)
            {
                this->manual();
            });

        main_vis_2 = std::make_shared<
            open3d::visualization::visualizer::O3DVisualizer>(
            "Open3D - Wycinek", WIDTH, HEIGHT);
        main_vis_2->AddAction(
            "set2",
            [this](open3d::visualization::visualizer::O3DVisualizer &)
            {
                this->get_sets2();
            });
        main_vis_2->AddAction(
            "manual",
            [this](open3d::visualization::visualizer::O3DVisualizer &)
            {
                this->manual();
            });

        main_vis_1->SetOnClose(
            [this]()
            { return this->OnMainWindowClosing1(); });
        main_vis_2->SetOnClose(
            [this]()
            { return this->OnMainWindowClosing2(); });

        // faktoryzacja
        source = open3d::io::CreatePointCloudFromFile(std::getenv("HOME") + PathScan + "/data/CornerMap.pcd");
        auto source_d = open3d::io::CreatePointCloudFromFile(std::getenv("HOME") + PathScan + "/data/SurfMap.pcd");
        source->PaintUniformColor(color_red);
        source_d->PaintUniformColor(color_blue);
        *source = *source + *source_d;

        target = open3d::io::CreatePointCloudFromFile(pcd);
        auto target_d = open3d::io::CreatePointCloudFromFile(pcd);
        target->PaintUniformColor(color_red);
        target_d->PaintUniformColor(color_yelloow);
        *target = *target + *target_d;
        // faktoryzacja

        double voxel_size = 0.4;

        source = source->VoxelDownSample(voxel_size);
        target = target->VoxelDownSample(voxel_size);

        open3d::visualization::gui::Application::GetInstance().AddWindow(
            main_vis_1);
        open3d::visualization::gui::Application::GetInstance().AddWindow(
            main_vis_2);

        main_vis_1->AddGeometry("mapa", source);
        main_vis_2->AddGeometry("wycinek", target);

        auto bounds_s = source->GetAxisAlignedBoundingBox();
        Eigen::Vector3f center_s = bounds_s.GetCenter().cast<float>();

        auto bounds_t = target->GetAxisAlignedBoundingBox();
        Eigen::Vector3f center_t = bounds_t.GetCenter().cast<float>();

        main_vis_1->SetupCamera(60, center_s, center_s + CENTER_OFFSET,
                                {0.0f, -1.0f, 0.0f});

        main_vis_2->SetupCamera(60, center_t, center_t + CENTER_OFFSET,
                                {0.0f, -1.0f, 0.0f});

        open3d::visualization::gui::Application::GetInstance().Run();
    }

private:
    void get_sets1()
    {
        std::cerr << "  set1   ";
        if (main_vis_1->GetSelectionSets().empty())
        {
            std::cerr << "Użyj zakładki Selection do wybrania punktów\n";
        }
        else
        {
            auto sets1 = main_vis_1->GetSelectionSets();

            auto &source_picked_set = sets1[0]["mapa"];

            std::vector<open3d::visualization::visualizer::
                            O3DVisualizerSelections::SelectedIndex>
                source_picked(source_picked_set.begin(),
                              source_picked_set.end());
            std::sort(source_picked.begin(), source_picked.end());
            source_pic_glob = source_picked;
            if (source_pic_glob.size() == 3)
            {
                std::cerr << "Pobrano 3 próbki\n";
            }
            else
            {
                std::cerr << "Zła ilość próbek (wymagana liczba próbek = 3, "
                             "twoja liczba = "
                          << source_pic_glob.size() << " )\n";
            }
        }
    }

    void get_sets2()
    {
        std::cerr << "  set2  ";

        if (main_vis_2->GetSelectionSets().empty())
        {
            std::cerr << "Użyj zakładki Selection do wybrania punktów\n";
        }
        else
        {
            auto sets2 = main_vis_2->GetSelectionSets();

            auto &target_picked_set = sets2[0]["wycinek"];

            std::vector<open3d::visualization::visualizer::
                            O3DVisualizerSelections::SelectedIndex>
                target_picked(target_picked_set.begin(),
                              target_picked_set.end());
            std::sort(target_picked.begin(), target_picked.end());
            target_pic_glob = target_picked;
            if (target_pic_glob.size() == 3)
            {
                std::cerr << "Pobrano 3 próbki\n";
            }
            else
            {
                std::cerr << "Zła ilość próbek (wymagana liczba próbek = 3, "
                             "twoja liczba = "
                          << target_pic_glob.size() << " )\n";
            }
        }
    }

    void manual()
    {
        std::cerr << "  manual  ";
        if (source_pic_glob.size() == 3 && target_pic_glob.size() == 3)
        {
            auto t = GetICPTransform(*source, *target, source_pic_glob,
                                     target_pic_glob);
            source->PaintUniformColor(color_blue);
            target->PaintUniformColor(color_yelloow);

            source->Transform(t);

            double zoom = 0.4559;
            Eigen::Vector3d front(0.6452, -0.3036, -0.7011);
            Eigen::Vector3d up(-0.2779, -0.9482, 0.1556);
            Eigen::Vector3d lookat(1.9892, 2.0208, 1.8945);

            open3d::visualization::DrawGeometries(
                {source, target}, "c++ open3d", 640, 480, 50, 50, false,
                false, false, &lookat, &up, &front, &zoom);
            std::cerr << "\n \n";
            std::cerr << "Macierz transformacji: \n";
            std::cerr << t;
            std::cerr << "\n \n";
        }
        else
        {
            std::cerr << "Niepoprawna ilość próbek \n";
            std::cerr << "Liczba próbek source = " << source_pic_glob.size()
                      << " (wymagane 3) \n";
            std::cerr << "Liczba próbek target = " << target_pic_glob.size()
                      << " (wymagane 3)\n";
        }
    }

    Eigen::Matrix4d_u GetICPTransform(
        const open3d::geometry::PointCloud &source,
        const open3d::geometry::PointCloud &target,
        const std::vector<open3d::visualization::visualizer::
                              O3DVisualizerSelections::SelectedIndex>
            &source_picked,
        const std::vector<open3d::visualization::visualizer::
                              O3DVisualizerSelections::SelectedIndex>
            &target_picked)
    {
        std::vector<Eigen::Vector2i> indices;

        for (size_t i = 0; i < source_picked.size(); ++i)
        {
            indices.emplace_back(source_picked[i].index,
                                 target_picked[i].index);
        }

        open3d::pipelines::registration::TransformationEstimationPointToPoint
            p2p;
        auto trans_init = p2p.ComputeTransformation(source, target, indices);

        const double threshold = 1.5;
        auto result = open3d::pipelines::registration::RegistrationICP(
            source, target, threshold, trans_init);

        return result.transformation_;
    }

    bool OnMainWindowClosing1()
    {
        main_vis_1.reset();
        return true;
    }

    bool OnMainWindowClosing2()
    {
        main_vis_2.reset();
        return true;
    }
    const std::string path_to_resources = std::getenv("HOME") + std::string("/open3d_install/bin/Open3D/resources");
    std::shared_ptr<open3d::visualization::visualizer::O3DVisualizer>
        main_vis_1;
    std::shared_ptr<open3d::visualization::visualizer::O3DVisualizer>
        main_vis_2;
    const int WIDTH = 1024;
    const int HEIGHT = 768;
    const Eigen::Vector3f CENTER_OFFSET = Eigen::Vector3f(0.0f, 0.0f, -40.0f);
    const Eigen::Vector3d color_blue =
        Eigen::Vector3d(0.0 / 255.0, 162 / 255.0, 255 / 255.0);
    const Eigen::Vector3d color_red =
        Eigen::Vector3d(148 / 255.0, 1 / 255.0, 1 / 255.0);
    const Eigen::Vector3d color_yelloow =
        Eigen::Vector3d(252 / 255.0, 194 / 255.0, 3 / 255.0);
    std::shared_ptr<open3d::geometry::PointCloud> source =
        std::make_shared<open3d::geometry::PointCloud>();
    std::shared_ptr<open3d::geometry::PointCloud> target =
        std::make_shared<open3d::geometry::PointCloud>();
    std::vector<open3d::visualization::visualizer::O3DVisualizerSelections::
                    SelectedIndex>
        source_pic_glob;
    std::vector<open3d::visualization::visualizer::O3DVisualizerSelections::
                    SelectedIndex>
        target_pic_glob;
};

auto preprocess_point_cloud(const open3d::geometry::PointCloud &pcd)
{
    std::shared_ptr<open3d::geometry::PointCloud> pcd_temp(new open3d::geometry::PointCloud);
    *pcd_temp = pcd;
    auto pcd_down = pcd_temp->VoxelDownSample(FeatureVoxelSize);
    pcd_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(NormalEstimationRadius, NormalKnn));
    auto pcd_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(*pcd_down,
                                                                        open3d::geometry::KDTreeSearchParamHybrid(FeatureRadius, FeatureKnn));
    return std::make_tuple(pcd_down, pcd_fpfh);
}
open3d::pipelines::registration::RegistrationResult execute_global_registration(const open3d::geometry::PointCloud &source_down,
                                                                                const open3d::pipelines::registration::Feature &source_fpfh,
                                                                                const open3d::geometry::PointCloud &target_down,
                                                                                const open3d::pipelines::registration::Feature &target_fpfh)
{
    std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>> correspondence_checker;
    auto correspondence_checker_edge_length = open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(CorrespondenceCheckerEdgeLenght);
    auto correspondence_checker_distance = open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(CorrespondenceCheckerDistance);
    correspondence_checker.push_back(correspondence_checker_edge_length);
    correspondence_checker.push_back(correspondence_checker_distance);
    auto result = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(source_down,
                                                                                            target_down, source_fpfh, target_fpfh, true, RansacMaxCorrespondenceDistanse,
                                                                                            open3d::pipelines::registration::TransformationEstimationPointToPoint(false), RansacModelSize,
                                                                                            correspondence_checker, open3d::pipelines::registration::RANSACConvergenceCriteria(RansacNumerIteration, RansacProbability));
    return result;
}
open3d::pipelines::registration::RegistrationResult execute_fast_global_registration(const open3d::geometry::PointCloud &source_down,
                                                                                     const open3d::pipelines::registration::Feature &source_fpfh,
                                                                                     const open3d::geometry::PointCloud &target_down,
                                                                                     const open3d::pipelines::registration::Feature &target_fpfh)
{
    auto option = open3d::pipelines::registration::FastGlobalRegistrationOption();
    option.maximum_correspondence_distance_ = CorrespondenceCheckerDistance;
    auto result = open3d::pipelines::registration::FastGlobalRegistrationBasedOnFeatureMatching(source_down, target_down,
                                                                                                source_fpfh, target_fpfh, option);
    return result;
}
open3d::pipelines::registration::RegistrationResult execute_ICP_registration(const open3d::geometry::PointCloud &source,
                                                                             const open3d::geometry::PointCloud &target,
                                                                             const Eigen::Matrix4d_u &transformation)
{
    auto criteria = open3d::pipelines::registration::ICPConvergenceCriteria();
    criteria.max_iteration_ = IcpNumIterParam;
    auto result = open3d::pipelines::registration::RegistrationICP(source, target, MaxIcpCorrespondenceDistance, transformation,
                                                                   open3d::pipelines::registration::TransformationEstimationPointToPlane(), criteria);
    return result;
}
bool prepareDataset()
{
    string path = std::getenv("HOME") + PathScan + "/data/GlobalMap.pcd";
    target_GlobalMap = open3d::io::CreatePointCloudFromFile(path);
    target_GlobalMap->PaintUniformColor(color_target);
    target_GlobalMap = target_GlobalMap->VoxelDownSample(0.8);
    std::shared_ptr<open3d::geometry::PointCloud> down, target;
    std::shared_ptr<open3d::pipelines::registration::Feature> fpfh;
    for (int i = 0; i < Scans; i++)
    {
        path = std::getenv("HOME") + PathScan + "/data/merged/" + std::to_string(i) + ".pcd";
        target = open3d::io::CreatePointCloudFromFile(path);
        std::tie(down, fpfh) = preprocess_point_cloud(*target);
        target_scans_down.push_back(std::move(down));
        target_scans_fpfh.push_back(std::move(fpfh));
    }
    return true;
}

bool CompareRegistration(const open3d::pipelines::registration::RegistrationResult& a, const open3d::pipelines::registration::RegistrationResult& b){
    if (a.fitness_ > b.fitness_) return true;
    if (a.fitness_ < b.fitness_) return false;
    
    if (a.inlier_rmse_ < b.inlier_rmse_) return true;
    if (a.inlier_rmse_ > b.inlier_rmse_) return false;
    
    return false;
}