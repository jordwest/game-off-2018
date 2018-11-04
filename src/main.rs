extern crate ggez;
extern crate nalgebra;
extern crate ncollide2d;
extern crate nphysics2d;

use ggez::graphics::Image;
use ggez::graphics::{DrawParam, Point2};
use ggez::*;
use nalgebra::zero;
use nalgebra::Isometry2;
use nalgebra::Vector2;
use nphysics2d::object::{BodyHandle, Material};
use nphysics2d::world::World;

const COLLIDER_MARGIN: f32 = 0.01;

struct MainState {
    world: World<f32>,
    handle: Option<BodyHandle>,
    image: Image,
}

impl MainState {
    fn new(ctx: &mut Context) -> GameResult<MainState> {
        use nphysics2d::solver::SignoriniCoulombPyramidModel;
        let mut state = MainState {
            world: World::new(),
            handle: None,
            image: Image::new(ctx, "/bot.png").unwrap(),
        };
        state.world.set_gravity(Vector2::y() * -9.81);
        state.add_ground();
        state
            .world
            .set_contact_model(SignoriniCoulombPyramidModel::new());
        state.add_character();
        Ok(state)
    }

    fn add_character(&mut self) {
        use ncollide2d::shape::{Ball, ShapeHandle};
        use nphysics2d::math::Isometry;
        use nphysics2d::volumetric::Volumetric;

        let ball = ShapeHandle::new(Ball::new(1.0));
        let local_inertia = ball.inertia(1.0);
        let local_center_of_mass = ball.center_of_mass();

        let handle = self.world.add_rigid_body(
            Isometry::new(Vector2::x() * 0.0, zero()),
            local_inertia,
            local_center_of_mass,
        );
        let _collider1 = self.world.add_collider(
            COLLIDER_MARGIN,
            ball,
            handle,
            Isometry2::identity(),
            Material::default(),
        );
        self.handle = Some(handle);
    }

    fn add_ground(&mut self) {
        use ncollide2d::shape::{Cuboid, ShapeHandle};

        let ground_radx = 1000.0;
        let ground_rady = 100.0;
        let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(
            ground_radx - COLLIDER_MARGIN,
            ground_rady - COLLIDER_MARGIN,
        )));

        let ground_pos = Isometry2::new(-Vector2::y() * 300.0, -0.05);
        self.world.add_collider(
            COLLIDER_MARGIN,
            ground_shape,
            BodyHandle::ground(),
            ground_pos,
            Material::default(),
        );
    }

    fn step(&mut self) {
        self.world.step();
    }
}

impl event::EventHandler for MainState {
    fn update(&mut self, _ctx: &mut Context) -> GameResult<()> {
        self.step();
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
        graphics::clear(ctx);
        let h = self.handle.unwrap();
        let body = self.world.rigid_body(h).unwrap();
        let isometry = body.position();
        let pos = isometry.translation.vector;
        let rot = isometry.rotation.angle();
        graphics::draw_ex(
            ctx,
            &self.image,
            DrawParam {
                dest: Point2::new(pos.x, -pos.y),
                rotation: -rot,
                offset: Point2::new(0.5, 0.5),
                scale: Point2::new(0.05, 0.05),
                ..DrawParam::default()
            },
        )
        .unwrap();
        graphics::present(ctx);
        Ok(())
    }
}

pub fn main() {
    let c = conf::Conf::new();
    c.window_mode.vsync(false);
    let ctx = &mut Context::load_from_conf("go2018", "jordwest", c).unwrap();
    let state = &mut MainState::new(ctx).unwrap();
    event::run(ctx, state).unwrap();
}
