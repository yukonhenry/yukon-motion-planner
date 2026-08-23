use sea_orm_migration::prelude::*;
use sea_orm_migration::schema::{json_binary, pk_auto, string};

pub struct Migration;

impl MigrationName for Migration {
    fn name(&self) -> &str {
        "m20260818_015346_create_robots"
    }
}

#[async_trait::async_trait]
impl MigrationTrait for Migration {
    async fn up(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        manager.create_table(
            Table::create()
                .table(Robots::Table)
                .if_not_exists()
                .col(pk_auto(Robots::Id))
                .col(string(Robots::Name))
                .col(json_binary(Robots::Capabilities))
                .to_owned(),
        ).await
    }
    async fn down(&self, manager: &SchemaManager) -> Result<(), DbErr> {
        manager.drop_table(
            Table::drop()
                .table(Robots::Table)
                .if_exists()
                .to_owned(),
        ).await
    }
}

#[derive(Iden)]
pub enum Robots {
    Table,
    Id,
    Name,
    Capabilities,
}




