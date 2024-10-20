package org.firstinspires.ftc.teamcode.feature.subsystem;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.wrapper.CachedMotor;

import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class PitchingSlides implements Subsystem {
    private Dependency<?> dependencies = Subsystem.DEFAULT_DEPENDENCY;
    public static final PitchingSlides INSTANCE = new PitchingSlides();
    private CachedMotor leftSlideMotor;
    private CachedMotor rightSideMotor;
    private CachedMotor angleMotor;

    private PitchingSlides() {

    }


    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependencies;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependencies = dependency;
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(java.lang.annotation.ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach {
    }

//    public Lambda pitchTo() {
//
//  }
    //public Lambda setSlidePowers() {

    //}

    //public Lambda setSlidePosition() {

    //}

    //public Lambda executeGrabSequence() {

   // }
}
