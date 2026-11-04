package org.firstinspires.ftc.teamcode.config.util

import com.pedropathing.api.Paths.line
import com.pedropathing.api.PoseFactory
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.interpolator.Interpolator
import org.firstinspires.ftc.teamcode.config.util.PoseUtil.Poses.Blue.bottomBlueFlower
import org.firstinspires.ftc.teamcode.config.util.PoseUtil.Poses.Blue.movePoseTest
import org.firstinspires.ftc.teamcode.config.util.PoseUtil.Poses.Blue.startPoseTest

object PoseUtil {
    val p: PoseFactory = PoseFactory.degrees()
    object Poses {
        object Blue {
            val startPoseTest: Pose = p.of(9.1, 9.2, 90.0)
            val movePoseTest: Pose = p.of(80.0, 92.0, 90.0)
            val movePoseTestHead: Pose = p.of(80.0, 92.0, 270.0)
            val bottomBlueFlower: Pose = p.of(95.5, 32.0, 270.0)
        }
        object Red {
            val bottomCellStartWall: Pose = p.of(55.0, 8.0, 90.0)
            val redSpike: Pose = p.of(8.0, 8.0, 180.0)
            val redSpikeControl1: Pose = p.of(65.0567, 20.7167, 0.0)
            val topCellSpikeScoring: Pose = p.of(26.8084, 119.3781, -50.0)
            val TopFlower: Pose = p.of(36.9637, 129.4173, 47.0)
            val topFlowerScore: Pose = p.of(33.5295, 126.318, -50.0)
            val bottomFlower: Pose = p.of(16.9047, 46.6075, 180.0)
            val bottomFlowerToBottomCell: Pose = p.of(55.0, 8.0, 90.0)
            val bottomFlowerToBottomCellSegment1Heading: Pose = p.of(55.0, 8.0, 180.0)
            val bottomFlowerToBottomCellSegment2Heading: Pose = p.of(55.0, 8.0, 90.0)
            val bottomCellFireToPark: Pose = p.of(10.4154, 87.9747, 90.0)
        }
    }
    object Paths {
        object Blue {
            fun testPath(): Path = line(startPoseTest, movePoseTest).linear(startPoseTest,movePoseTest)
            fun testPathToFlower(): Path = line(movePoseTest, bottomBlueFlower).constant(movePoseTest)
        }
        object Red {
            fun bottomCellWallToRedSpike(): Path {
                return com.pedropathing.api.Paths.curve(
                    PoseUtil.Poses.Red.bottomCellStartWall,
                    PoseUtil.Poses.Red.redSpikeControl1,
                    PoseUtil.Poses.Red.redSpike
                )
                .heading(
                    Interpolator.piecewise()
                        .until(0.4, Interpolator.constant(90.0))
                        .until(1.0, Interpolator.constant(180.0))
                )
            }
            fun redSpikeToTopScoring(): Path {
                return line(
                    PoseUtil.Poses.Red.redSpike,
                    PoseUtil.Poses.Red.topCellSpikeScoring
                )
                .heading(
                    Interpolator.piecewise()
                        .until(0.8, Interpolator.constant(PoseUtil.Poses.Red.redSpike))
                        .until(1.0, Interpolator.constant(PoseUtil.Poses.Red.topCellSpikeScoring))
                )
            }
            fun topSpikeScoringToTopFlower(): Path {
                return line(
                    PoseUtil.Poses.Red.topCellSpikeScoring,
                    PoseUtil.Poses.Red.TopFlower
                )
                .constant(
                    PoseUtil.Poses.Red.TopFlower
                )
            }
            fun topFlowerToFlowerScore(): Path {
                return line(
                    PoseUtil.Poses.Red.TopFlower,
                    PoseUtil.Poses.Red.topFlowerScore
                )
                .linear(
                    PoseUtil.Poses.Red.TopFlower,
                    PoseUtil.Poses.Red.topFlowerScore
                )
            }
            fun topFlowerScoreToBottomFlower(): Path {
                return line(
                    PoseUtil.Poses.Red.topFlowerScore,
                    PoseUtil.Poses.Red.bottomFlower
                )
                .constant(
                    PoseUtil.Poses.Red.bottomFlower
                )
            }
            fun bottomFlowerToBottomCell(): Path {
                return line(
                    PoseUtil.Poses.Red.bottomFlower,
                    PoseUtil.Poses.Red.bottomFlowerToBottomCell
                )
                .heading(
                    Interpolator.piecewise()
                        .until(0.5, Interpolator.constant(180.0))
                        .until(1.0, Interpolator.constant(90.0))
                )
            }
            fun bottomCellFireToPark(): Path {
                return line(
                    PoseUtil.Poses.Red.bottomFlowerToBottomCell,
                    PoseUtil.Poses.Red.bottomCellFireToPark
                )
                .constant(
                   PoseUtil.Poses.Red.bottomCellFireToPark
                )
            }
        }
    }
}