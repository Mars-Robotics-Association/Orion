package org.firstinspires.ftc.teamcode._RobotCode.Juan;

import java.util.Objects;

public interface FunctionStandIn<T, R> {
    R apply(T t);

    default <V> FunctionStandIn<T, V> andThen(FunctionStandIn<? super R, ? extends V> after) {
        Objects.requireNonNull(after);
        return (T t) -> after.apply(apply(t));
    }
}
