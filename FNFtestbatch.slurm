#!/bin/bash
## This is a comment
## Name of your job                                                            
#SBATCH --job-name=8NOFemaleMC
## Memory that you need
#SBATCH --time=15:00:00                                             
#SBATCH --mem-per-cpu=10G
## Partition name: no need to change                                           
#SBATCH --partition=compute
## Run N jobs, M at a time: 1-N%M                                             
#SBATCH --array=1-30%30
## Your email                                                        
#SBATCH --mail-user=farzana.rahman@oist.jp
## Working directory                                                 
#SBATCH --workdir=/work/DoyaU/Farzana/MCTEST/MC8                             
#SBATCH --input=none                                                           
#SBATCH --output=/work/DoyaU/Farzana/MCTEST/MC8/clogs/matlab_job_%j.out             
## Load matlab (see 'module avail' for a list of available matlab versions)    
module load matlab/R2014b
## matlab execution command : dont change -nojvm                                                   
matlab_exe="matlab -nosplash -nodisplay -nodesktop"
## parameterized matlab script execution command                               
szarr=(1	2	3	4	5	6	7	8	9	10	11	12	13	14	15	16	17	18	19	20	21	22	23	24	25	26	27	28	29	30	31	32	33	34	35	36	37	38	39	40	41	42	43	44	45	46	47	48	49	50	51	52	53	54	55	56	57	58	59	60	61	62	63	64	65	66	67	68	69	70	71	72	73	74	75	76	77	78	79	80	81	82	83	84	85	86	87	88	89	90	91	92	93	94	95	96	97	98	99	100	101	102	103	104	105	106	107	108	109	110	111	112	113	114	115	116	117	118	119	120	121	122	123	124	125	126	127	128	129	130	131	132	133	134	135	136	137	138	139	140	141	142	143	144	145	146	147	148	149	150)
## setting matlab parameter
xs=${szarr[$(($SLURM_ARRAY_TASK_ID - 1))]}
## Matlab command. Make sure no "clear all;" is in the script
mtalab_cmd="x=${xs}; NFmatlabMainF"
## run matlab                                                                  
${matlab_exe} -r "${mtalab_cmd}"

## log completed job in .err file                                              
>&2 echo "[${SLURM_JOB_NAME}_${SLURM_JOB_ID} completed]"
