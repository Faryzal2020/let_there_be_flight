public class FlightModeAutomatic extends FlightModeStandard {

  @runtimeProperty("ModSettings.mod", "Let There Be Flight")
  @runtimeProperty("ModSettings.category", "UI-Settings-Flight-Mode-Settings")
  @runtimeProperty("ModSettings.displayName", "UI-Settings-Automatic-Mode-Enabled")
  public let enabled: Bool = false;

  @runtimeProperty("ModSettings.mod", "Let There Be Flight")
  @runtimeProperty("ModSettings.category", "UI-Settings-Automatic-Mode")
  @runtimeProperty("ModSettings.displayName", "UI-Settings-Auto-Braking-Factor")
  @runtimeProperty("ModSettings.description", "UI-Settings-Auto-Braking-Factor-Description")
  @runtimeProperty("ModSettings.step", "0.1")
  @runtimeProperty("ModSettings.min", "0")
  @runtimeProperty("ModSettings.max", "10")
  public let automaticModeAutoBrakingFactor: Float = 3.0;

  @runtimeProperty("ModSettings.mod", "Let There Be Flight")
  @runtimeProperty("ModSettings.category", "UI-Settings-Automatic-Mode")
  @runtimeProperty("ModSettings.displayName", "UI-Settings-Yaw-Directionality")
  @runtimeProperty("ModSettings.description", "UI-Settings-Yaw-Directionality-Description")
  @runtimeProperty("ModSettings.step", "1.0")
  @runtimeProperty("ModSettings.min", "0")
  @runtimeProperty("ModSettings.max", "1000")
  public let automaticModeYawDirectionality: Float = 300.0;

  // How fast the lift input moves the held hover height. Automatic mode used to borrow
  // Standard mode's Lift Factor for this; it now has its own.
  @runtimeProperty("ModSettings.mod", "Let There Be Flight")
  @runtimeProperty("ModSettings.category", "UI-Settings-Automatic-Mode")
  @runtimeProperty("ModSettings.displayName", "UI-Settings-Automatic-Lift-Factor")
  @runtimeProperty("ModSettings.description", "UI-Settings-Automatic-Lift-Factor-Description")
  @runtimeProperty("ModSettings.step", "0.5")
  @runtimeProperty("ModSettings.min", "0.5")
  @runtimeProperty("ModSettings.max", "50.0")
  public let automaticModeLiftFactor: Float = 25.0;

  // Gain on the whole lift force in Automatic mode. It cannot go to zero: at zero the
  // vehicle gets no lift at all and simply falls.
  @runtimeProperty("ModSettings.mod", "Let There Be Flight")
  @runtimeProperty("ModSettings.category", "UI-Settings-Automatic-Mode")
  @runtimeProperty("ModSettings.displayName", "UI-Settings-Automatic-Hover-Factor")
  @runtimeProperty("ModSettings.description", "UI-Settings-Automatic-Hover-Factor-Description")
  @runtimeProperty("ModSettings.step", "1.0")
  @runtimeProperty("ModSettings.min", "5.0")
  @runtimeProperty("ModSettings.max", "100.0")
  public let automaticModeHoverFactor: Float = 40.0;

  protected let hovering: Float;
  protected let referenceZ: Float;
  protected let gravityTrim: Float;
  protected let isBrakingVertical: Bool;
  protected let brakeDirection: Float;
  protected let brakeTimer: Float;

  public static func Create(component: ref<FlightComponent>) -> ref<FlightModeAutomatic> {
    let self = new FlightModeAutomatic();
    self.Initialize(component);
    self.hovering = 1.0;
    self.gravityTrim = 0.0;
    self.isBrakingVertical = false;
    self.brakeDirection = 0.0;
    self.brakeTimer = 0.0;
    return self;
  }

  public func Initialize(component: ref<FlightComponent>) -> Void {
    super.Initialize(component);
    this.collisionPenalty = 1.0;
  }

  public func Activate() -> Void {
    let normal: Vector4;
    this.referenceZ = this.component.stats.d_position.Z;
    this.gravityTrim = 0.0;
    this.isBrakingVertical = false;
    this.brakeDirection = 0.0;
    this.brakeTimer = 0.0;
    this.component.FindGround(normal);
    this.component.hoverHeight = MaxF(this.component.distance, FlightSettings.GetFloat("hoverModeMinHoverHeight"));
  }

  public func Deactivate() {
    this.gravityTrim = 0.0;
    this.isBrakingVertical = false;
    this.brakeDirection = 0.0;
    this.brakeTimer = 0.0;
  }

  public func GetDescription() -> String = "Automatic";

  public func Update(timeDelta: Float) -> Void {
    let lastHovering = this.hovering;
    let normal: Vector4;
    let foundGround = this.component.FindGround(normal);
    if foundGround {
      this.hovering = ClampF(1.0 - (this.component.distance - FlightSettings.GetFloat("hoverModeMinHoverHeight")) / (FlightSettings.GetFloat("hoverModeMaxHoverHeight") - FlightSettings.GetFloat("hoverModeMinHoverHeight")), 0.0, 1.0);
    } else {
      this.hovering = 0.0;
    }

    let isLiftActive = AbsF(this.component.lift) > 0.05;

    if lastHovering == 0.0 && this.hovering > 0.0 {
      this.component.hoverHeight = MaxF(this.component.distance + this.component.lift * timeDelta * this.automaticModeLiftFactor, FlightSettings.GetFloat("hoverModeMinHoverHeight"));
    } else {
      this.component.hoverHeight = MaxF(this.component.hoverHeight + this.component.lift * timeDelta * this.automaticModeLiftFactor, FlightSettings.GetFloat("hoverModeMinHoverHeight"));
    }

    let heightDifference = this.component.hoverHeight - this.component.distance;
    let idealNormal = Vector4.Interpolate(FlightUtils.Up(), normal, this.hovering);

    let hoverCorrection = this.component.hoverGroundPID.GetCorrectionClamped(heightDifference, timeDelta, FlightSettings.GetFloat("hoverClamp"));// / FlightSettings.GetFloat("hoverClamp");

    let vz = this.component.stats.d_velocity.Z;
    let airCorrection = 0.0;

    if isLiftActive || this.hovering > 0.0 {
      this.isBrakingVertical = false;
      this.brakeTimer = 0.0;
      this.referenceZ = this.component.stats.d_position.Z;
      airCorrection = this.component.lift;
      if this.component.lift > 0.05 {
        this.brakeDirection = 1.0;
      } else if this.component.lift < -0.05 {
        this.brakeDirection = -1.0;
      } else {
        this.brakeDirection = 0.0;
      }
    } else {
      if this.brakeDirection != 0.0 {
        this.isBrakingVertical = true;
      }

      if this.isBrakingVertical {
        this.brakeTimer += timeDelta;
        let stoppedAscent: Bool = this.brakeDirection > 0.0 && vz <= 0.05;
        let stoppedDescent: Bool = this.brakeDirection < 0.0 && vz >= -0.05;
        if stoppedAscent || stoppedDescent || AbsF(vz) <= 0.05 || this.brakeTimer > 2.0 {
          this.isBrakingVertical = false;
          this.brakeDirection = 0.0;
          this.brakeTimer = 0.0;
          this.referenceZ = this.component.stats.d_position.Z;
          airCorrection = 0.0;
        } else {
          airCorrection = -ClampF(vz * 0.12, -0.6, 0.6);
        }
      }

      if !this.isBrakingVertical {
        let zDiff = this.referenceZ - this.component.stats.d_position.Z;
        this.gravityTrim = ClampF(this.gravityTrim + (vz - zDiff * 0.2) * timeDelta * 0.4, -4.0, 4.0);
        airCorrection = ClampF(zDiff * 0.15 - vz * 0.1, -0.4, 0.4);
      }
    }

    let liftFactor = LerpF(this.hovering, airCorrection, hoverCorrection);
    let effectiveGravity = (9.81000042 * this.gravityFactor) - (this.gravityTrim * (1.0 - this.hovering));

    this.UpdateWithNormalLift(timeDelta, idealNormal, liftFactor * this.automaticModeHoverFactor + effectiveGravity);

    let aeroFactor = Vector4.Dot(this.component.stats.d_forward, this.component.stats.d_direction);
    let yawDirectionality: Float = this.component.stats.d_speedRatio * this.automaticModeYawDirectionality;

    let directionFactor = AbsF(Vector4.Dot(this.component.stats.d_forward - this.component.stats.d_direction, this.component.stats.d_right));

    this.force += FlightUtils.Forward() * directionFactor * yawDirectionality * aeroFactor;
    this.force += -this.component.stats.d_localDirection * directionFactor * yawDirectionality * AbsF(aeroFactor);

    if AbsF(this.component.surge) < 1.0 {
      // let velocityDamp: Vector4 = (1.0 - AbsF(this.component.surge)) * this.automaticModeAutoBrakingFactor * this.component.stats.d_localDirection2D * (this.component.stats.d_speed2D / 100.0);
      // this.force -= velocityDamp;
      let velocityDamp: Vector4 = (1.0 - AbsF(this.component.surge)) * this.automaticModeAutoBrakingFactor * this.component.stats.s_brakingFrictionFactor * this.component.stats.d_localVelocity;
      this.force.X -= velocityDamp.X;
      this.force.Y -= velocityDamp.Y;
    }
  }
}
