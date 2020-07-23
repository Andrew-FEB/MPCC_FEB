import opengen as og

problem = og.builder.Problem(u_seq, x0, cost) \
          .with_aug_lagrangian_constraints(F1, C) \
          .with_constraints(U)
