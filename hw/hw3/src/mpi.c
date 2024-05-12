#include <stdio.h>
#include <stdlib.h>
#include <mpi.h>
#include <string.h>
#include "matrix.h"

int main(int argc, char *argv[])
{
    /** Matrix Properties
     * [0] = Rows of Matrix A
     * [1] = Cols of Matrix A
     * [2] = Rows of Matrix B
     * [3] = Cols of Matrix B
     **/
    //printf("B1\n");
    int matrix_properties[4];

    int num_worker, rank;
    double start_time, end_time;
    double *data_A = NULL, *data_B = NULL,*data_C=NULL;
    double *blk_A, *res_blk;

    matrix_struct *result_matrix=NULL,*m_1=NULL,*m_2=NULL;

    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &num_worker);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    //printf("B2\n");

    /** the master initializes the data **/
    if (rank == 0)
    {

        if (argc != 3)
        {
            printf("ERROR: Please specify only 2 files.\n");
            exit(EXIT_FAILURE);
        }

        m_1 = get_continuous_matrix_struct(argv[1]);
        m_2 = get_continuous_matrix_struct(argv[2]);
        //printf("B3\n");

        if (m_1->cols != m_2->rows)
        {
            printf("ERROR: The number of columns of matrix A must match the number of rows of matrix B"
                    "(%d ,%d).\n",m_1->cols,m_2->rows);
            exit(EXIT_FAILURE);
        }

        // fill the property-array for workers
        //printf("B4\n");
        matrix_properties[0] = m_1->rows;
        matrix_properties[1] = m_1->cols;
        matrix_properties[2] = m_2->rows;
        matrix_properties[3] = m_2->cols;
        //printf("B5\n");
        start_time = MPI_Wtime();

        // TODO
        data_A = &(m_1->mat_data[0][0]);
        data_B = &(m_2->mat_data[0][0]);
        //printf("B6\n");
        result_matrix=malloc(sizeof(matrix_struct));
        result_matrix->rows = matrix_properties[0];
        result_matrix->cols = matrix_properties[1];
        //printf("%ld\n", result_matrix->cols * result_matrix->rows * sizeof(double));
        data_C=malloc(result_matrix->cols*result_matrix->rows*sizeof(double));
        //printf("B7\n");
        result_matrix->mat_data = malloc(result_matrix->rows*sizeof(double *));
        for (int i = 0; i < result_matrix->rows; ++i)
            result_matrix->mat_data[i] = &(data_C[result_matrix->cols*i]);
        //printf("A\n");
    }
    MPI_Barrier( MPI_COMM_WORLD);
    MPI_Bcast(matrix_properties,
              4,
              MPI_INT,
              0,
              MPI_COMM_WORLD);
    //printf("B\n");
    int per_blk_row = matrix_properties[0] / num_worker;
    int blk_A_size = per_blk_row * matrix_properties[1];
    int mat_B_size = matrix_properties[2] * matrix_properties[3];
    if (rank != 0)
    {
        data_B = calloc(mat_B_size, sizeof(double));
    }
    blk_A = calloc(blk_A_size, sizeof(double));
    res_blk = calloc(blk_A_size, sizeof(double));
    MPI_Scatter(data_A,
                blk_A_size,
                MPI_DOUBLE,
                blk_A,
                blk_A_size,
                MPI_DOUBLE,
                0,
                MPI_COMM_WORLD);
    //printf("C\n");
    MPI_Bcast(data_B,
              mat_B_size,
              MPI_DOUBLE,
              0,
              MPI_COMM_WORLD);
    //printf("D\n");
    for (int i = 0; i < per_blk_row; ++i)
        for (int j = 0; j < matrix_properties[3]; ++j)
            for (int k = 0; k <matrix_properties[1]; ++k)
                res_blk[i * matrix_properties[3] + j] += blk_A[i * matrix_properties[3]+k]*data_B[k*matrix_properties[3]+j];
    //printf("E\n");
    MPI_Gather(res_blk, 
               blk_A_size,
               MPI_DOUBLE,
               data_C,
               blk_A_size,
               MPI_DOUBLE,0,
               MPI_COMM_WORLD);
    //printf("F\n");

    free(blk_A);
    free(res_blk);
    /** The master presents the results on the console */
    if (rank == 0)
    {
        free_continuous_matrix(m_1);
        free_continuous_matrix(m_2);
        free_continuous_matrix(result_matrix);
        end_time = MPI_Wtime();
        printf("Matrix rows: %d; MPI Execution time: %f seconds; Worker num: %d\n",matrix_properties[0], end_time - start_time,num_worker);
    }
    else
    {
        free(data_B);
    }

    MPI_Finalize();
    exit(EXIT_SUCCESS);
}
