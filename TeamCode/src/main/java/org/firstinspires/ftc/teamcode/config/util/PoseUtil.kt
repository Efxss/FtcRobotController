package org.firstinspires.ftc.teamcode.config.util

import com.pedropathing.api.Paths.curve
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
            // Bottom Auto Two and a half
            val bottomCellStartWall: Pose = p.of(55.0, 8.0, 90.0)
            var bottomCellPullOut: Pose = p.of(55.0, 16.0, 90.0)
            val spike: Pose = p.of(7.0, 7.5, 180.0)
            val spikeControl1: Pose = p.of(65.0567, 20.7167, 0.0)
            val topCellSpikeScoring: Pose = p.of(26.8084, 119.3781, -50.0)
            val TopFlower: Pose = p.of(36.9637, 129.4173, 47.0)
            val topFlowerScore: Pose = p.of(33.5295, 126.318, -50.0)
            val bottomFlower: Pose = p.of(16.9047, 46.6075, 180.0)
            val bottomFlowerToBottomCell: Pose = p.of(55.0, 8.0, 90.0)
            val bottomFlowerToBottomCellSegment1Heading: Pose = p.of(55.0, 8.0, 180.0)
            val bottomFlowerToBottomCellSegment2Heading: Pose = p.of(55.0, 8.0, 90.0)
            val bottomCellFireToPark: Pose = p.of(10.4154, 87.9747, 90.0)

            // Top Auto Fire Park
            val topRedBlueFlowerSideStart:Pose = p.of(58.4, 133.4, 270.0)
            val topRedBlueFlowerSideToPark:Pose = p.of(15.0829, 125.145, 270.0)
            val topRedBlueFlowerSideToParkControl1:Pose = p.of(62.5904, 102.3984, 0.0)
        }
    }
    object Paths {
        object Blue {
            fun testPath(): Path = line(startPoseTest, movePoseTest).linear(startPoseTest,movePoseTest)
            fun testPathToFlower(): Path = line(movePoseTest, bottomBlueFlower).constant(movePoseTest)
        }
        object Red {
            // Bottom Auto Two and a half
            fun bottomCellWallOutFire(): Path {
                return line(
                    Poses.Red.bottomCellStartWall,
                    Poses.Red.bottomCellPullOut
                )
                .constant(
                    Poses.Red.bottomCellPullOut
                )
            }
            fun bottomCellOutToRedSpike(): Path {
                return line(
                    Poses.Red.bottomCellPullOut,
                    Poses.Red.spike
                )
                .constant(
                    Poses.Red.spike
                )
            }
            fun redSpikeToTopScoring(): Path {
                return line(
                    Poses.Red.spike,
                    Poses.Red.topCellSpikeScoring
                )
                .heading(
                    Interpolator.piecewise()
                        .until(0.8, Interpolator.constant(Poses.Red.spike))
                        .until(1.0, Interpolator.constant(Poses.Red.topCellSpikeScoring))
                )
            }
            fun topSpikeScoringToTopFlower(): Path {
                return line(
                    Poses.Red.topCellSpikeScoring,
                    Poses.Red.TopFlower
                )
                .constant(
                    Poses.Red.TopFlower
                )
            }
            fun topFlowerToFlowerScore(): Path {
                return line(
                    Poses.Red.TopFlower,
                    Poses.Red.topFlowerScore
                )
                .linear(
                    Poses.Red.TopFlower,
                    Poses.Red.topFlowerScore
                )
            }
            fun topFlowerScoreToBottomFlower(): Path {
                return line(
                    Poses.Red.topFlowerScore,
                    Poses.Red.bottomFlower
                )
                .constant(
                    Poses.Red.bottomFlower
                )
            }
            fun bottomFlowerToBottomCell(): Path {
                return line(
                    Poses.Red.bottomFlower,
                    Poses.Red.bottomFlowerToBottomCell
                )
                .heading(
                    Interpolator.piecewise()
                        .until(0.5, Interpolator.constant(180.0))
                        .until(1.0, Interpolator.constant(90.0))
                )
            }
            fun bottomCellFireToPark(): Path {
                return line(
                    Poses.Red.bottomFlowerToBottomCell,
                    Poses.Red.bottomCellFireToPark
                )
                .constant(
                   Poses.Red.bottomCellFireToPark
                )
            }
            // Top Auto Fire Park
            fun topRedBlueFlowerSideToPark(): Path {
                return curve(
                    Poses.Red.topRedBlueFlowerSideStart,
                    Poses.Red.topRedBlueFlowerSideToParkControl1,
                    Poses.Red.topRedBlueFlowerSideToPark
                )
                .constant(
                    Poses.Red.topRedBlueFlowerSideToPark,
                )
            }
        }
    }
}