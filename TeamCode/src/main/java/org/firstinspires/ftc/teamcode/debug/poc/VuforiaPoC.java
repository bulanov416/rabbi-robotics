package org.firstinspires.ftc.teamcode.debug.poc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.vuforia.*;

/**
 * Created by Levi on 10/30/2016
 */
@TeleOp(name = "VuforiaPoc")
public class VuforiaPoC extends LinearOpMode {

    DcMotor l;
    DcMotor r;
    DcMotor lb;
    DcMotor rb;
    VuforiaLocalizer vuforia;

    public VuforiaPoC() {

    }

    @Override
    public void runOpMode() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AbbyKYb/////AAAAGdY6wDr3Z0XtvJTxXpU8Q5U61MqZpMKu6CD+Pf4PY/rpg/SFuQluL91sRhc0UghpMKfC142dhhpDwqcjHnDxMCzQUDZL9niCNdAGdUVO5awB2mjCbtQqOm/OyzzQNsYRggcgFHMeXY+qGwquhTlsJOBPuH5p3+KfCzXfMKUTw7oX52HpkKtpI/tqfhSOUTpu27SLI86R2PgdFMOsZ6syIDkLVeTuMvrh+Zi0/rcyS6sKNZ81Z/rYCxc2rdDq8gWx/b7rt3+MImO9dSmjYSeQV76zkCKf2+jQpjIWBjMOIJm7azDl+l8vojno+YESCzyJxajl7Vp/f55id4uZwi7wg2YZ/3MqIc/gvWctSoMsa6cT";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");
        rb = hardwareMap.dcMotor.get("rb");
        lb = hardwareMap.dcMotor.get("lb");
        while (true) {

            r.setDirection(DcMotor.Direction.REVERSE);
            rb.setDirection(DcMotor.Direction.REVERSE);



        }
    }
}