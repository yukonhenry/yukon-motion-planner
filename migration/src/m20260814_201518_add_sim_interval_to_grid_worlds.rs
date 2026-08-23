use crate::m20260727_180611_create_grid_worlds::GridWorlds;
use sea_orm_migration::prelude::*;

pub struct Migration;

impl MigrationName for Migration {
    fn name(&self) -> &str {
        "m20260814_201518_add_sim_interval_to_grid_worlds"
    }
}

#[async_trait::async_trait]
impl MigrationTrait for Migration {
    /// How often, in seconds, the environment task perturbs this grid's dynamic obstacles.
    ///
    /// `double` rather than `decimal`: the entity reads it as `f64`, and Postgres `NUMERIC`
    /// comes back as a `Decimal` that will not deserialize into one. Sub-second intervals are
    /// the interesting ones — a simulation at 10 Hz is `0.1` — so an integer column would not
    /// do either.
    ///
    /// Defaulted rather than nullable so every existing row is immediately runnable, and so
    /// "how fast does this grid move" always has an answer rather than a null to interpret.
    async fn up(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        manager
            .alter_table(
                Table::alter()
                    .table(GridWorlds::Table)
                    .add_column(
                        ColumnDef::new(GridWorlds::SimInterval)
                            .double()
                            .not_null()
                            .default(1.0),
                    )
                    .to_owned(),
            )
            .await
    }

    async fn down(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        manager
            .alter_table(
                Table::alter()
                    .table(GridWorlds::Table)
                    .drop_column(GridWorlds::SimInterval)
                    .to_owned(),
            )
            .await
    }
}
